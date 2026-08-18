#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/geometric_controller.hpp"

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/mode_executor.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

static const std::string kModeName = "MPC Controller";
static const std::string kNodeName = "px4_attitude_mode_node";

class MpcFlightMode : public px4_ros2::ModeBase
{
public:
  explicit MpcFlightMode(rclcpp::Node & node)
  : ModeBase(
      node,
      Settings{"MPC Controller"}
        .activateEvenWhileDisarmed(true)
        .preventArming(false)),
    node_(node)
  {
    attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

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
          RCLCPP_INFO(node_.get_logger(), "Mission completed signal received! Handoff to PX4 Landing...");
          completed(px4_ros2::Result::Success);
        }
      });

    RCLCPP_INFO(node_.get_logger(),
      "MpcFlightMode initialized: registered with PX4 Flight Mode Manager");
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "MpcFlightMode ACTIVATED");
  }

  void onDeactivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "MpcFlightMode DEACTIVATED");
  }

  void updateSetpoint(float dt_s) override
  {
    (void)dt_s;
    std::lock_guard<std::mutex> lock(mutex_);
    sendSetpoint();
  }

  void sendSetpoint()
  {
    if (!latest_setpoint_ || !last_setpoint_time_) {
      // Stream level attitude and neutral hover thrust while waiting for MPC
      const Eigen::Quaternionf q_level = Eigen::Quaternionf::Identity();
      const Eigen::Vector3f thrust_frd(0.f, 0.f, -static_cast<float>(hover_thrust_));
      attitude_setpoint_->update(q_level, thrust_frd);
      return;
    }

    const double age = (node_.now() - *last_setpoint_time_).seconds();
    if (age > 0.5) {
      // Stale setpoint: command level hover
      const Eigen::Quaternionf q_level = Eigen::Quaternionf::Identity();
      const Eigen::Vector3f thrust_frd(0.f, 0.f, -static_cast<float>(hover_thrust_));
      attitude_setpoint_->update(q_level, thrust_frd);
      return;
    }

    // Convert desired quaternion from body FLU -> world ENU (ROS) to body FRD -> world NED (PX4)
    const auto &q_flu_enu_raw = latest_setpoint_->desired_attitude_wxyz;
    Eigen::Quaterniond q_flu_enu(q_flu_enu_raw[0], q_flu_enu_raw[1], q_flu_enu_raw[2], q_flu_enu_raw[3]);
    const auto q_frd_ned_opt = mpc_controller::px4_control::fluEnuToFrdNed(q_flu_enu);

    Eigen::Quaternionf q_frd_ned = Eigen::Quaternionf::Identity();
    if (q_frd_ned_opt) {
      q_frd_ned = q_frd_ned_opt->cast<float>();
    }

    // Compute normalized collective thrust: T_norm = hover_thrust * (collective_specific_force / g)
    constexpr double gravity = 9.80665;
    const double specific_force = latest_setpoint_->desired_collective_specific_force_m_s2;
    double thrust_norm = hover_thrust_ * (specific_force / gravity);
    thrust_norm = std::clamp(thrust_norm, 0.05, 0.95);

    // Body FRD Z thrust is negative (e.g. [0, 0, -T])
    const Eigen::Vector3f thrust_frd(0.f, 0.f, -static_cast<float>(thrust_norm));

    attitude_setpoint_->update(q_frd_ned, thrust_frd);
  }

private:
  rclcpp::Node & node_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  rclcpp::Subscription<mpc_controller::msg::ForceAttitudeSetpoint>::SharedPtr force_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_completed_sub_;
  std::optional<mpc_controller::msg::ForceAttitudeSetpoint> latest_setpoint_;
  std::optional<rclcpp::Time> last_setpoint_time_;
  double hover_thrust_ = 0.60;
  std::mutex mutex_;
};

class MpcModeExecutor : public px4_ros2::ModeExecutorBase
{
public:
  enum class State
  {
    Idle,
    Arming,
    TakingOff,
    MpcRunning,
    Landing,
    Disarming,
    Done
  };

  explicit MpcModeExecutor(px4_ros2::ModeBase & owned_mode)
  : ModeExecutorBase(
      px4_ros2::ModeExecutorBase::Settings{}.activate(
        px4_ros2::ModeExecutorBase::Settings::Activation::ActivateAlways),
      owned_mode),
    node_(owned_mode.node())
  {
    start_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "mpc_mode_executor/start",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(node_.get_logger(), "[Executor] Start service requested");
        if (state_ != State::Idle && state_ != State::Done) {
          response->success = false;
          response->message = "Executor is currently active in state: " + std::to_string(static_cast<int>(state_));
          return;
        }
        runState(State::Arming, px4_ros2::Result::Success);
        response->success = true;
        response->message = "MpcModeExecutor started: Arming -> Takeoff -> MPC -> Land";
      });

    land_service_ = node_.create_service<std_srvs::srv::Trigger>(
      "mpc_mode_executor/land",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        RCLCPP_INFO(node_.get_logger(), "[Executor] Land service requested");
        runState(State::Landing, px4_ros2::Result::Success);
        response->success = true;
        response->message = "Landing sequence initiated";
      });
    start_mission_client_ = node_.create_client<std_srvs::srv::Trigger>(
      "/reference_generator_node/start_mission");
  }

  void onActivate() override
  {
    RCLCPP_INFO(node_.get_logger(), "MpcModeExecutor activated: initiating autonomous sequence");
    runState(State::Arming, px4_ros2::Result::Success);
  }

  void onDeactivate(DeactivateReason reason) override
  {
    (void)reason;
    RCLCPP_WARN(node_.get_logger(), "MpcModeExecutor deactivated");
    state_ = State::Idle;
  }

  void runState(State state, px4_ros2::Result previous_result)
  {
    if (previous_result != px4_ros2::Result::Success) {
      RCLCPP_ERROR(
        node_.get_logger(), "State %d: previous step failed (%s)",
        static_cast<int>(state), resultToString(previous_result));
      state_ = State::Idle;
      return;
    }

    state_ = state;
    switch (state_) {
      case State::Idle:
      case State::Done:
        break;

      case State::Arming:
        RCLCPP_INFO(node_.get_logger(), "[Executor] Step 1/5: Arming vehicle...");
        arm([this](px4_ros2::Result result) {
          runState(State::TakingOff, result);
        });
        break;

      case State::TakingOff:
        RCLCPP_INFO(node_.get_logger(), "[Executor] Step 2/5: Taking off to altitude 10.0m...");
        takeoff([this](px4_ros2::Result result) {
          runState(State::MpcRunning, result);
        }, 10.0f);
        break;

      case State::MpcRunning:
        RCLCPP_INFO(node_.get_logger(), "[Executor] Step 3/5: Drone reached 10m. Triggering reference mission...");
        if (start_mission_client_) {
          auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
          start_mission_client_->async_send_request(req);
        }
        RCLCPP_INFO(node_.get_logger(), "[Executor] Switching to MPC Attitude Flight Mode...");
        scheduleMode(
          ownedMode().id(), [this](px4_ros2::Result result) {
            RCLCPP_INFO(
              node_.get_logger(),
              "[Executor] MPC flight mode finished (%s), proceeding to land...",
              resultToString(result));
            runState(State::Landing, px4_ros2::Result::Success);
          });
        break;

      case State::Landing:
        RCLCPP_INFO(node_.get_logger(), "[Executor] Step 4/5: Landing vehicle...");
        land([this](px4_ros2::Result result) {
          runState(State::Disarming, result);
        });
        break;

      case State::Disarming:
        RCLCPP_INFO(node_.get_logger(), "[Executor] Step 5/5: Waiting until vehicle is disarmed...");
        waitUntilDisarmed([this](px4_ros2::Result result) {
          RCLCPP_INFO(node_.get_logger(), "[Executor] Vehicle disarmed. Autonomous sequence complete!");
          runState(State::Done, result);
        });
        break;
    }
  }

private:
  rclcpp::Node & node_;
  State state_{State::Idle};
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr land_service_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_mission_client_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<px4_ros2::NodeWithModeExecutor<MpcModeExecutor, MpcFlightMode>>(kNodeName, false);
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::cerr << "px4_attitude_mode_node exception: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
