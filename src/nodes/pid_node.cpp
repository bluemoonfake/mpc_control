#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/compare/pid.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <stdexcept>
#include <utility>

// PX4 owns position/velocity feedback. This node only samples the shared
// reference and transports feed-forward setpoints; no PID gains live here.
class PidFlightMode final : public px4_ros2::ModeBase {
  using Clock = std::chrono::steady_clock;
  using Message = mpc_controller::msg::ReferenceTrajectory;
  using Reference = mpc_controller::pid_validation::Reference;
  using ValidatedReference = mpc_controller::pid_validation::ValidatedReference;
public:
  explicit PidFlightMode(rclcpp::Node &node): ModeBase(node, Settings{"PX4 PID"}.preventArming(true)), node_(node) {
    timeout_ = node.declare_parameter("reference_timeout_seconds", 1.5);
    frame_ = node.declare_parameter<std::string>("frame_id", "map");

    if (!std::isfinite(timeout_) || timeout_ <= 0 || frame_.empty())
      throw std::invalid_argument("Invalid PID reference timeout/frame");
    setpoint_ = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);

    setSetpointUpdateRate(50.0f);

    ref_sub_ = node.create_subscription<Message>("reference_trajectory", rclcpp::QoS(10), [this](Message::ConstSharedPtr msg) {
          const double stamp = rclcpp::Time(msg->header.stamp).seconds();
          Reference ref;
          ref.header_time_seconds = stamp;
          ref.hold_after_end = msg->hold_after_end;
          ref.limits.max_speed_xy = msg->max_speed_xy;
          ref.limits.max_speed_z = msg->max_speed_z;
          ref.limits.max_acceleration_xy = msg->max_acceleration_xy;
          ref.limits.max_acceleration_z = msg->max_acceleration_z;
          ref.limits.max_control_rate_xy = msg->max_control_rate_xy;
          ref.limits.max_control_rate_z = msg->max_control_rate_z;
          for (const auto &p : msg->points) {
            if (p.time_from_start.nanosec >= 1000000000U) { reference_.reset(); return; }
            ref.points.push_back({p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                                  p.position, p.velocity, p.acceleration, p.yaw, p.yaw_rate});
          }
          ValidatedReference validated;
          if (msg->header.frame_id != frame_ || stamp <= 0 || (last_stamp_ > 0 && stamp < last_stamp_) ||
              !validated.assign(std::move(ref))) {
            reference_.reset();
            return;
          }
          last_stamp_ = stamp;
          reference_ = std::move(validated);
          received_ = Clock::now();
        });
    external_mode_state_publisher_ = node.create_publisher<std_msgs::msg::Bool>(
        mpc_controller::mission::kPx4PidExternalModeStateTopic.data(),
        rclcpp::QoS(1).reliable().transient_local());
    publishModeState(false);
  }

  void onActivate() override {
    publishModeState(true);
    RCLCPP_INFO(node_.get_logger(), "PX4 PID active; mission start remains operator-controlled");
  }
  void onDeactivate() override { publishModeState(false); }

  void checkArmingAndRunConditions(px4_ros2::HealthAndArmingCheckReporter &reporter) override {
    if (!sample()) reporter.armingCheckFailureExt(0x70696401, px4_ros2::events::Log::Error, "PID reference unavailable or stale");
  }

  void updateSetpoint(float) override {
    if (!isActive()) return;
    const auto value = sample();
    if (!value) {
      // Report failure to PX4's mode manager; never manufacture a zero target.
      completed(px4_ros2::Result::ModeFailureOther);
      return;
    }
    px4_ros2::TrajectorySetpoint sp;
    sp.withPosition(value->position).withVelocity(value->velocity).withAcceleration(value->acceleration).withYaw(value->yaw).withYawRate(value->yaw_rate);
    setpoint_->update(sp);
  }
private:
  void publishModeState(bool active) {
    std_msgs::msg::Bool message;
    message.data = active;
    external_mode_state_publisher_->publish(message);
  }

  std::optional<mpc_controller::pid_validation::Sample> sample() {
    const double now = node_.now().seconds();
    if (last_now_ > 0 && now < last_now_) reference_.reset();
    last_now_ = now;
    if (!reference_) return {};
    return mpc_controller::pid_validation::sampleNed(*reference_, now,std::chrono::duration<double>(Clock::now() - received_).count(), timeout_);
  }
  rclcpp::Node &node_;
  double timeout_, last_stamp_{0}, last_now_{0};
  std::string frame_;
  std::optional<ValidatedReference> reference_;
  Clock::time_point received_{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> setpoint_;
  rclcpp::Subscription<Message>::SharedPtr ref_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr external_mode_state_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PidFlightMode>>("pid_mode_node"));
  } catch (const std::exception &e) {
    RCLCPP_ERROR(rclcpp::get_logger("pid_mode_node"), "%s", e.what());
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
