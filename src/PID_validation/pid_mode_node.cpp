#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/px4/pid_reference.hpp"
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <chrono>
#include <stdexcept>

// PX4 owns position/velocity feedback. This node only samples the shared
// reference and transports feed-forward setpoints; no PID gains live here.
class PidFlightMode final : public px4_ros2::ModeBase {
  using Clock = std::chrono::steady_clock;
  using Message = mpc_controller::msg::ReferenceTrajectory;
  using Reference = mpc_controller::pid_validation::Reference;
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
          for (const auto &p : msg->points) {
            if (p.time_from_start.nanosec >= 1000000000U) { reference_.reset(); return; }
            ref.points.push_back({p.time_from_start.sec + p.time_from_start.nanosec * 1e-9,
                                  p.position, p.velocity, p.acceleration, p.yaw, p.yaw_rate});
          }
          if (msg->header.frame_id != frame_ || stamp <= 0 || (last_stamp_ > 0 && stamp < last_stamp_) ||
              !mpc_controller::translational::ReferenceSampler::validTrajectory(ref)) {
            reference_.reset();
            return;
          }
          last_stamp_ = stamp;
          reference_ = std::move(ref);
          received_ = Clock::now();
        });
  }

  void onActivate() override {
    RCLCPP_INFO(node_.get_logger(), "PX4 PID active; mission start remains operator-controlled");
  }
  void onDeactivate() override {}

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
  std::optional<Reference> reference_;
  Clock::time_point received_{};
  std::shared_ptr<px4_ros2::TrajectorySetpointType> setpoint_;
  rclcpp::Subscription<Message>::SharedPtr ref_sub_;
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
