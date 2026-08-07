#include "mpc_controller/msg/direct_acceleration_command.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"

#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <mutex>
#include <optional>
#include <functional>
#include <utility>

class MpcControllerNode final : public rclcpp::Node
{
public:
  MpcControllerNode()
  : Node("mpc_controller_node")
  {
    declare_parameter("update_rate_hz", update_rate_hz_);
    declare_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("update_rate_hz", update_rate_hz_);
    get_parameter("reference_timeout_seconds", reference_timeout_seconds_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);

    reference_subscription_ = create_subscription<Reference>(
        "reference_trajectory", 10,
        [this](Reference::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          reference_ = std::move(*message);
          reference_received_at_ = get_clock()->now();
        });
    state_subscription_ = create_subscription<State>(
        "vehicle_state", 10,
        [this](State::SharedPtr message) {
          std::lock_guard<std::mutex> lock(mutex_);
          state_ = std::move(*message);
          state_received_at_ = get_clock()->now();
        });
    command_publisher_ = create_publisher<Command>("direct_acceleration_command", 10);
    timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(1.0 / update_rate_hz_)),
        std::bind(&MpcControllerNode::update, this));
  }

private:
  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using State = mpc_controller::msg::VehicleState;
  using Command = mpc_controller::msg::DirectAccelerationCommand;
  static constexpr int horizon = 8;

  struct Sample
  {
    std::array<double, 3> p{};
    std::array<double, 3> v{};
    std::array<double, 3> a{};
    double yaw = 0.0;
    double yaw_rate = 0.0;
  };

  static double durationSeconds(const builtin_interfaces::msg::Duration& d)
  {
    return static_cast<double>(d.sec) + static_cast<double>(d.nanosec) * 1.0e-9;
  }

  template<typename Point>
  static Sample interpolate(const Point& left, const Point& right, double alpha)
  {
    Sample result;
    for (int axis = 0; axis < 3; ++axis) {
      result.p[axis] = left.position[axis] + alpha * (right.position[axis] - left.position[axis]);
      result.v[axis] = left.velocity[axis] + alpha * (right.velocity[axis] - left.velocity[axis]);
      result.a[axis] = left.acceleration[axis] + alpha * (right.acceleration[axis] - left.acceleration[axis]);
    }
    result.yaw = left.yaw + alpha * (right.yaw - left.yaw);
    result.yaw_rate = left.yaw_rate + alpha * (right.yaw_rate - left.yaw_rate);
    return result;
  }

  static bool sampleAt(const Reference& reference, double time, Sample& output)
  {
    if (reference.points.empty()) {
      return false;
    }
    if (time <= durationSeconds(reference.points.front().time_from_start)) {
      output = interpolate(reference.points.front(), reference.points.front(), 0.0);
      return true;
    }
    for (std::size_t i = 1; i < reference.points.size(); ++i) {
      const double left_time = durationSeconds(reference.points[i - 1].time_from_start);
      const double right_time = durationSeconds(reference.points[i].time_from_start);
      if (time <= right_time) {
        const double alpha = (time - left_time) / (right_time - left_time);
        output = interpolate(reference.points[i - 1], reference.points[i], alpha);
        return true;
      }
    }
    output = interpolate(reference.points.back(), reference.points.back(), 0.0);
    return reference.hold_after_end;
  }

  static double cost(double p, double v, const double* rp, const double* rv,
      const double* ra, double input, double previous) noexcept
  {
    constexpr double dt = 0.02;
    double predicted_p = p;
    double predicted_v = v;
    double result = 0.0;
    for (int i = 0; i < horizon; ++i) {
      predicted_p += dt * predicted_v + 0.5 * dt * dt * input;
      predicted_v += dt * input;
      const double ep = predicted_p - rp[i];
      const double ev = predicted_v - rv[i];
      const double ea = input - ra[i];
      result += 12.0 * ep * ep + 2.0 * ev * ev + 0.25 * ea * ea;
    }
    const double du = input - previous;
    return result + 0.04 * input * input + 2.0 * du * du;
  }

  void update()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!reference_ || !state_ || !state_->valid || !state_->position_valid
        || !state_->velocity_valid || !state_->acceleration_valid
        || !state_->yaw_valid) {
      return;
    }
    const auto now = get_clock()->now();
    if ((now - reference_received_at_).seconds() > reference_timeout_seconds_
        || (now - state_received_at_).seconds() > state_timeout_seconds_) {
      return;
    }
    const double elapsed = (now - rclcpp::Time(reference_->header.stamp)).seconds();
    if (!std::isfinite(elapsed) || elapsed < 0.0) {
      return;
    }

    std::array<std::array<double, horizon>, 3> rp{};
    std::array<std::array<double, horizon>, 3> rv{};
    std::array<std::array<double, horizon>, 3> ra{};
    Sample first;
    for (int step = 0; step < horizon; ++step) {
      Sample sample;
      if (!sampleAt(reference_.value(), elapsed + 0.02 * step, sample)) {
        return;
      }
      if (step == 0) {
        first = sample;
      }
      for (int axis = 0; axis < 3; ++axis) {
        rp[axis][step] = sample.p[axis];
        rv[axis][step] = sample.v[axis];
        ra[axis][step] = sample.a[axis];
      }
    }

    double command_acceleration[3]{};
    for (int axis = 0; axis < 3; ++axis) {
      const double previous = previous_acceleration_[axis];
      const double lower = std::max(-4.0, previous - 0.25);
      const double upper = std::min(4.0, previous + 0.25);
      double best = previous;
      double best_cost = INFINITY;
      for (int candidate_index = 0; candidate_index <= 8; ++candidate_index) {
        const double candidate = lower + (upper - lower) * candidate_index / 8.0;
        const double candidate_cost = cost(
            state_->position[axis], state_->velocity[axis], rp[axis].data(),
            rv[axis].data(), ra[axis].data(), candidate, previous);
        if (candidate_cost < best_cost) {
          best = candidate;
          best_cost = candidate_cost;
        }
      }
      command_acceleration[axis] = best;
      previous_acceleration_[axis] = best;
    }

    Command command;
    command.header.stamp = now;
    command.header.frame_id = "map";
    command.trajectory_id = reference_->trajectory_id;
    command.sequence = ++sequence_;
    command.acceleration = {command_acceleration[0], command_acceleration[1], command_acceleration[2]};
    command.yaw = first.yaw;
    command.yaw_rate = first.yaw_rate;
    command.valid = true;
    command_publisher_->publish(command);
  }

  std::mutex mutex_;
  std::optional<Reference> reference_;
  std::optional<State> state_;
  rclcpp::Time reference_received_at_{0, 0, RCL_ROS_TIME};
  rclcpp::Time state_received_at_{0, 0, RCL_ROS_TIME};
  double update_rate_hz_ = 50.0;
  double reference_timeout_seconds_ = 0.5;
  double state_timeout_seconds_ = 0.25;
  std::array<double, 3> previous_acceleration_{0.0, 0.0, 0.0};
  uint64_t sequence_ = 0;
  rclcpp::Subscription<Reference>::SharedPtr reference_subscription_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Publisher<Command>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcControllerNode>());
  rclcpp::shutdown();
  return 0;
}
