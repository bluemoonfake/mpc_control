#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/px4/geometric_mapping.hpp"

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

static const std::string kNodeName = "px4_attitude_mode_node";

class MpcFlightMode : public px4_ros2::ModeBase {
public:
  explicit MpcFlightMode(rclcpp::Node &node)
      : ModeBase(node, Settings{"MPC Controller"}
                           .preventArming(true)),  // mode can be activated while disarm
        node_(node) {
    attitude_setpoint_ =std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    force_setpoint_sub_ = node.create_subscription<mpc_controller::msg::ForceAttitudeSetpoint>(
            "force_attitude_setpoint", rclcpp::QoS(10),
            [this](const mpc_controller::msg::ForceAttitudeSetpoint::SharedPtr msg) {
              if (!msg) {
                return;
              }
              std::lock_guard<std::mutex> lock(mutex_);
              latest_setpoint_ = *msg;
              last_setpoint_time_ = node_.now();
              sendSetpoint();
            });

    hover_thrust_sub_ = node.create_subscription<px4_msgs::msg::HoverThrustEstimate>(
            "/fmu/out/hover_thrust_estimate", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
              if (msg && msg->valid && std::isfinite(msg->hover_thrust) && msg->hover_thrust > 0.05f) {
                std::lock_guard<std::mutex> lock(mutex_);
                hover_thrust_ = msg->hover_thrust;
              }
            });

    mission_completed_sub_ = node.create_subscription<std_msgs::msg::Bool>(
        "/reference_generator_node/mission_completed", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg && msg->data) {
            RCLCPP_INFO(
                node_.get_logger(),
                "Mission completed signal received; completing External Mode. "
                "Landing remains operator/PX4-mode-manager owned.");
            completed(px4_ros2::Result::Success);
          }
        });

    RCLCPP_INFO(
        node_.get_logger(),
        "MpcFlightMode initialized: registered with PX4 Flight Mode Manager");
  }

  void onActivate() override {
    RCLCPP_INFO(node_.get_logger(), "MpcFlightMode ACTIVATED");
  }

  void onDeactivate() override {
    RCLCPP_INFO(node_.get_logger(), "MpcFlightMode DEACTIVATED");
  }

  void updateSetpoint(float dt_s) override {
    (void)dt_s;
    std::lock_guard<std::mutex> lock(mutex_);
    sendSetpoint();
  }

  void sendSetpoint() {
    if (!latest_setpoint_ || !last_setpoint_time_) {
      // Stream level attitude and neutral hover thrust while waiting for MPC
      const Eigen::Quaternionf q_level = Eigen::Quaternionf::Identity();
      const Eigen::Vector3f thrust_frd(0.f, 0.f,-static_cast<float>(hover_thrust_));
      attitude_setpoint_->update(q_level, thrust_frd);
      return;
    }

    const double age = (node_.now() - *last_setpoint_time_).seconds();
    if (age > 0.5) {
      // Stale setpoint: command level hover
      const Eigen::Quaternionf q_level = Eigen::Quaternionf::Identity();
      const Eigen::Vector3f thrust_frd(0.f, 0.f,-static_cast<float>(hover_thrust_));
      attitude_setpoint_->update(q_level, thrust_frd);
      return;
    }

    // Convert desired quaternion from body FLU -> world ENU (ROS) to body FRD
    // -> world NED (PX4)
    const auto &q_flu_enu_raw = latest_setpoint_->desired_attitude_wxyz;
    Eigen::Quaterniond q_flu_enu(q_flu_enu_raw[0], q_flu_enu_raw[1],
                                 q_flu_enu_raw[2], q_flu_enu_raw[3]);
    const auto q_frd_ned_opt =
        mpc_controller::px4_control::fluEnuToFrdNed(q_flu_enu);

    Eigen::Quaternionf q_frd_ned = Eigen::Quaternionf::Identity();
    if (q_frd_ned_opt) {
      q_frd_ned = q_frd_ned_opt->cast<float>();
    }

    // Compute normalized collective thrust: T_norm = hover_thrust *
    // (collective_specific_force / g)
    constexpr double gravity = 9.80665;
    const double specific_force = latest_setpoint_->desired_collective_specific_force_m_s2;
    double thrust_norm = hover_thrust_ * (specific_force / gravity);
    thrust_norm = std::clamp(thrust_norm, 0.05, 0.95);

    // Body FRD Z thrust is negative (e.g. [0, 0, -T])
    const Eigen::Vector3f thrust_frd(0.f, 0.f,-static_cast<float>(thrust_norm));

    const auto yaw_rate_ned = mpc_controller::px4_control::enuYawRateToNed(latest_setpoint_->desired_yaw_rate_rad_s);
    if (!yaw_rate_ned) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,"Rejecting non-finite ENU yaw-rate setpoint");
      return;
    }

    attitude_setpoint_->update(q_frd_ned, thrust_frd,static_cast<float>(*yaw_rate_ned));
  }

private:
  rclcpp::Node &node_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  rclcpp::Subscription<mpc_controller::msg::ForceAttitudeSetpoint>::SharedPtr
      force_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr
      hover_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_completed_sub_;
  std::optional<mpc_controller::msg::ForceAttitudeSetpoint> latest_setpoint_;
  std::optional<rclcpp::Time> last_setpoint_time_;
  double hover_thrust_ = 0.60;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<px4_ros2::NodeWithMode<MpcFlightMode>>(
        kNodeName, false);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    std::cerr << "px4_attitude_mode_node exception: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
