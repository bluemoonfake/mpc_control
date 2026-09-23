#include "mpc_controller/application/mission_runtime.hpp"
#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/domain/mission/trajectory.hpp"
#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/srv/load_and_start_mission.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace
{

constexpr auto kMissionServiceName =
  "/reference_generator_node/load_and_start_mission";

int runMissionClient(const std::string &controller, const std::string &mission_path)
{
  int client_argc = 1;
  char client_name[] = "reference_generator_node";
  char *client_argv[] = {client_name, nullptr};
  rclcpp::init(client_argc, client_argv);

  auto node = rclcpp::Node::make_shared("runtime_mission_client");
  auto client = node->create_client<mpc_controller::srv::LoadAndStartMission>(
    kMissionServiceName);

  int result = 5;
  if (!client->wait_for_service(std::chrono::seconds(5))) {
    std::cerr << "Mission service unavailable: " << kMissionServiceName << '\n';
  } else {
    auto request = std::make_shared<mpc_controller::srv::LoadAndStartMission::Request>();
    request->controller = controller;
    request->mission_path = mission_path;
    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(
          node, future, std::chrono::seconds(10)) !=
      rclcpp::FutureReturnCode::SUCCESS) {
      std::cerr << "Mission service call timed out or failed\n";
    } else {
      const auto response = future.get();
      std::cout << response->message << '\n';
      result = response->success ? 0 : 4;
    }
  }

  rclcpp::shutdown();
  return result;
}

}  // namespace

class ReferenceGeneratorNode final : public rclcpp::Node {
public:
  ReferenceGeneratorNode() : Node("reference_generator_node") {
    declareAndGet("frame_id", frame_id_);
    declareAndGet("mission_acceptance_radius_m", acceptance_radius_m_);
    declareAndGet("mission_acceptance_speed_m_s", acceptance_speed_m_s_);
    declareAndGet("mission_speed_override_m_s", speed_override_m_s_);
    declareAndGet("hold_yaw_rad", hold_yaw_rad_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("horizon_seconds", horizon_seconds_);
    declareAndGet("sample_period_seconds", sample_period_seconds_);
    declareAndGet("planner_max_speed_xy_m_s", planner_max_speed_xy_m_s_);
    declareAndGet("planner_max_speed_z_m_s", planner_max_speed_z_m_s_);
    declareAndGet("planner_max_acceleration_xy_m_s2",
                  planner_max_acceleration_xy_m_s2_);
    declareAndGet("planner_max_acceleration_z_m_s2",
                  planner_max_acceleration_z_m_s2_);
    declareAndGet("planner_max_jerk_xy_m_s3", planner_max_jerk_xy_m_s3_);
    declareAndGet("planner_max_jerk_z_m_s3", planner_max_jerk_z_m_s3_);
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("visualization_enabled", visualization_enabled_);
    declareAndGet("visualization_publish_rate_hz",
                  visualization_publish_rate_hz_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    getVectorParameter("hold_position", hold_position_);

    valid_config_ =
        !frame_id_.empty() && positiveFinite(acceptance_radius_m_) &&
        positiveFinite(acceptance_speed_m_s_) &&
        std::isfinite(speed_override_m_s_) && speed_override_m_s_ >= 0.0 &&
        positiveFinite(horizon_seconds_) &&
        positiveFinite(sample_period_seconds_) &&
        positiveFinite(planner_max_speed_xy_m_s_) &&
        positiveFinite(planner_max_speed_z_m_s_) &&
        positiveFinite(planner_max_acceleration_xy_m_s2_) &&
        positiveFinite(planner_max_acceleration_z_m_s2_) &&
        positiveFinite(planner_max_jerk_xy_m_s3_) &&
        positiveFinite(planner_max_jerk_z_m_s3_) &&
        positiveFinite(publish_rate_hz_) &&
        positiveFinite(state_timeout_seconds_) &&
        horizon_seconds_ >= sample_period_seconds_;
    if (!valid_config_) {
      RCLCPP_ERROR(
          get_logger(),
          "Invalid reference generator parameters; publishing disabled");
      return;
    }

    mpc_controller::mission::MissionTrajectory::Config config;
    config.reference.hold_position = hold_position_;
    config.reference.hold_yaw_rad = hold_yaw_rad_;
    config.acceptance_radius_m = acceptance_radius_m_;
    config.acceptance_speed_m_s = acceptance_speed_m_s_;
    config.speed_override_m_s = speed_override_m_s_;
    config.horizon_seconds = horizon_seconds_;
    config.sample_period_seconds = sample_period_seconds_;
    config.planner_max_speed_xy_m_s = planner_max_speed_xy_m_s_;
    config.planner_max_speed_z_m_s = planner_max_speed_z_m_s_;
    config.planner_max_acceleration_xy_m_s2 =
        planner_max_acceleration_xy_m_s2_;
    config.planner_max_acceleration_z_m_s2 = planner_max_acceleration_z_m_s2_;
    config.planner_max_jerk_xy_m_s3 = planner_max_jerk_xy_m_s3_;
    config.planner_max_jerk_z_m_s3 = planner_max_jerk_z_m_s3_;
    mission_runtime_ =
        std::make_unique<mpc_controller::application::MissionRuntime>(config);

    publisher_ = create_publisher<Reference>("reference_trajectory", 10);
    mission_completed_publisher_ = create_publisher<std_msgs::msg::Bool>(
        "/reference_generator_node/mission_completed", 10);
    if (visualization_enabled_) {
      visualization_publisher_ =
          create_publisher<visualization_msgs::msg::MarkerArray>(
              "~/visualization_markers", 10);
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    state_subscription_ = create_subscription<State>(
        "vehicle_state", qos,
        std::bind(&ReferenceGeneratorNode::stateCallback, this,
                  std::placeholders::_1));
    const auto mode_state_qos = rclcpp::QoS(1).reliable().transient_local();
    mpc_mode_state_subscription_ = createModeStateSubscription(
        mpc_controller::mission::kMpcExternalModeStateTopic,
        mpc_controller::mission::kMpcController, mode_state_qos);
    px4_pid_mode_state_subscription_ = createModeStateSubscription(
        mpc_controller::mission::kPx4PidExternalModeStateTopic,
        mpc_controller::mission::kPx4PidController, mode_state_qos);
    load_and_start_mission_service_ =
        create_service<mpc_controller::srv::LoadAndStartMission>(
            "~/load_and_start_mission",
            [this](const mpc_controller::srv::LoadAndStartMission::Request::
                       SharedPtr request,
                   mpc_controller::srv::LoadAndStartMission::Response::SharedPtr
                       response) {
              const auto result = loadAndStartMission(request->controller,
                                                      request->mission_path);
              response->success = result.success;
              response->message = result.message;
            });
    start_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_mission",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = false;
          response->message =
              "Use /reference_generator_node/load_and_start_mission with "
              "the mission path.";
        });
    reset_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_mission",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          transitionToMeasuredHold("Mission reset");
          response->success = true;
          response->message = "Mission reset; holding the latest measured pose";
        });

    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)));
    timer_ = create_wall_timer(
        period, std::bind(&ReferenceGeneratorNode::publish, this));
    RCLCPP_INFO(
        get_logger(),
        "Reference generator initialized without a mission; rate=%.1f Hz",
        publish_rate_hz_);
  }

private:
  using Reference = mpc_controller::msg::ReferenceTrajectory;
  using Point = mpc_controller::msg::TrajectoryPoint;
  using State = mpc_controller::msg::VehicleState;
  using SteadyClock = std::chrono::steady_clock;

  template <typename T> void declareAndGet(const std::string &name, T &value) {
    declare_parameter(name, value);
    get_parameter(name, value);
  }

  static bool positiveFinite(double value) noexcept {
    return std::isfinite(value) && value > 0.0;
  }

  static double steadySeconds() noexcept {
    return std::chrono::duration<double>(SteadyClock::now().time_since_epoch())
        .count();
  }

  void getVectorParameter(const std::string &name,
                          std::array<double, 3> &value) {
    const auto vector = get_parameter(name).as_double_array();
    if (vector.size() == 3 &&
        std::all_of(vector.begin(), vector.end(),
                    [](double element) { return std::isfinite(element); })) {
      value = {vector[0], vector[1], vector[2]};
    }
  }

  static builtin_interfaces::msg::Duration
  durationMessage(uint64_t nanoseconds) noexcept {
    builtin_interfaces::msg::Duration duration;
    duration.sec = static_cast<int32_t>(nanoseconds / 1000000000ULL);
    duration.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000ULL);
    return duration;
  }

  mpc_controller::mission::RuntimeMissionLoader::Result
  loadAndStartMission(const std::string &controller,
                      const std::string &mission_path) {
    if (!mpc_controller::mission::isKnownController(controller)) {
      return rejectMissionStart("unsupported controller: " + controller);
    }
    if (active_controller_ != controller) {
      return rejectMissionStart("requested controller is not the active "
                                "PX4 External Mode");
    }

    // A request addressed to the active mode always replaces the previous
    // trajectory. Readiness and mission-validation failures therefore end in
    // measured-position hold, never the old trajectory.
    transitionToMeasuredHold("Mission replacement requested");
    if (!hasFreshControlReadyState()) {
      return rejectMissionStart(
          "fresh control-ready vehicle state is unavailable");
    }
    const auto result =
        mission_runtime_->loadAndStart(mission_path, steadySeconds());
    if (!result.success) {
      transitionToMeasuredHold("Mission request rejected");
      RCLCPP_ERROR(get_logger(), "Mission request rejected: %s",
                   result.message.c_str());
      return result;
    }
    const auto &target = mission_runtime_->trajectory().waypoints().front();
    RCLCPP_INFO(
        get_logger(),
        "Mission started: %zu waypoints, target='%s' [%.2f, %.2f, %.2f]",
        mission_runtime_->trajectory().waypoints().size(), target.id.c_str(), target.position[0],
        target.position[1], target.position[2]);
    return result;
  }

  mpc_controller::mission::RuntimeMissionLoader::Result
  rejectMissionStart(const std::string &message) {
    RCLCPP_ERROR(get_logger(), "Mission request rejected: %s", message.c_str());
    return {false, message};
  }

  bool hasFreshControlReadyState() const {
    return last_state_received_at_ && last_state_control_ready_ &&
           std::chrono::duration<double>(SteadyClock::now() -
                                         *last_state_received_at_)
                   .count() <= state_timeout_seconds_;
  }

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
  createModeStateSubscription(std::string_view topic,
                              std::string_view controller,
                              const rclcpp::QoS &qos) {
    return create_subscription<std_msgs::msg::Bool>(
        topic.data(), qos,
        [this, controller](const std_msgs::msg::Bool::SharedPtr message) {
          if (message) {
            externalModeStateCallback(controller, message->data);
          }
        });
  }

  void externalModeStateCallback(std::string_view controller, bool active) {
    if (active) {
      if (active_controller_ == controller) {
        return;
      }
      active_controller_ = controller;
      transitionToMeasuredHold("External Mode activation");
      return;
    }
    if (active_controller_ != controller) {
      return;
    }
    active_controller_.clear();
    transitionToMeasuredHold("External Mode deactivation");
  }

  void transitionToMeasuredHold(const char *reason) {
    mission_runtime_->abortToHold();
    pending_hold_capture_ = true;
    hold_reference_captured_ = false;
    captureHoldIfFresh();
    RCLCPP_INFO(get_logger(), "%s; waiting for measured-position hold", reason);
  }

  void captureHoldIfFresh() {
    if (!pending_hold_capture_ || !hasFreshControlReadyState()) {
      return;
    }
    mission_runtime_->trajectory().captureHoldFromVehicle();
    pending_hold_capture_ = false;
    hold_reference_captured_ = true;
    RCLCPP_INFO(get_logger(), "Measured hold captured");
  }

  void stateCallback(const State::SharedPtr message) {
    last_state_received_at_ = SteadyClock::now();
    mpc_controller::mission::MissionTrajectory::VehicleState state;
    state.position = {message->position[0], message->position[1],
                      message->position[2]};
    state.velocity = {message->velocity[0], message->velocity[1],
                      message->velocity[2]};
    state.yaw = message->yaw;
    state.valid = message->control_ready;
    last_state_control_ready_ = state.valid;
    mission_runtime_->trajectory().updateVehicleState(state);

    // Before the PX4 External Mode is selected, continuously follow the
    // admitted vehicle pose. Activation freezes that already-current target.
    if (state.valid && active_controller_.empty()) {
      mission_runtime_->trajectory().captureHoldFromVehicle();
      hold_reference_captured_ = true;
    } else if (state.valid && pending_hold_capture_) {
      captureHoldIfFresh();
    }
  }

  void publish() {
    if (!valid_config_ || !publisher_ || !mission_runtime_) {
      return;
    }
    const auto steady_now = SteadyClock::now();
    if (!last_state_received_at_ ||
        std::chrono::duration<double>(steady_now - *last_state_received_at_)
                .count() > state_timeout_seconds_) {
      mission_runtime_->trajectory().invalidateVehicleState();
    }
    const auto update = mission_runtime_->trajectory().update(
        std::chrono::duration<double>(steady_now.time_since_epoch()).count());
    if (update.waypoint_reached) {
      const auto &waypoint =
          mission_runtime_->trajectory().waypoints()[update.reached_waypoint_index];
      RCLCPP_INFO(
          get_logger(),
          "Waypoint %zu/%zu reached: id='%s', distance=%.2fm, crossed_plane=%s",
          update.reached_waypoint_index + 1, mission_runtime_->trajectory().waypoints().size(),
          waypoint.id.c_str(), update.distance_to_waypoint_m,
          update.crossed_finish_plane ? "yes" : "no");
    }
    if (update.mission_completed) {
      std_msgs::msg::Bool completed;
      completed.data = true;
      mission_completed_publisher_->publish(completed);
      RCLCPP_INFO(get_logger(), "Mission completed; holding the final mission position");
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    if (!update.samples.empty()) {
      const auto &limits = update.samples.front();
      message.max_speed_xy = limits.max_speed_xy;
      message.max_speed_z = limits.max_speed_z;
      message.max_acceleration_xy = limits.max_acceleration_xy;
      message.max_acceleration_z = limits.max_acceleration_z;
      message.max_control_rate_xy = limits.max_control_rate_xy;
      message.max_control_rate_z = limits.max_control_rate_z;
    }
    message.points.reserve(update.samples.size());
    const uint64_t sample_nanoseconds = static_cast<uint64_t>(sample_period_seconds_ * 1.0e9);
    for (std::size_t index = 0; index < update.samples.size(); ++index) {
      const auto &sample = update.samples[index];
      Point point;
      point.time_from_start = durationMessage(index * sample_nanoseconds);
      point.position = sample.position;
      point.velocity = sample.velocity;
      point.acceleration = sample.acceleration;
      point.yaw = sample.yaw;
      point.yaw_rate = sample.yaw_rate;
      message.points.push_back(point);
    }
    publisher_->publish(message);
    last_reference_ = message;

    if (visualization_enabled_ && visualization_publisher_) {
      publishVisualization();
    }
  }

  void publishVisualization() {
    const auto now = SteadyClock::now();
    if (last_visualization_published_at_ &&
        std::chrono::duration<double>(now - *last_visualization_published_at_)
                .count() <
            1.0 / std::max(visualization_publish_rate_hz_, 1.0)) {
      return;
    }
    last_visualization_published_at_ = now;
    visualization_msgs::msg::MarkerArray markers;

    visualization_msgs::msg::Marker path;
    path.header.stamp = get_clock()->now();
    path.header.frame_id = frame_id_;
    path.ns = "mission_path";
    path.id = 0;
    path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path.action = visualization_msgs::msg::Marker::ADD;
    path.scale.x = 0.15;
    path.color.g = 0.8F;
    path.color.b = 0.2F;
    path.color.a = 0.8F;
    for (const auto &waypoint : mission_runtime_->trajectory().waypoints()) {
      geometry_msgs::msg::Point point;
      point.x = waypoint.position[0];
      point.y = waypoint.position[1];
      point.z = waypoint.position[2];
      path.points.push_back(point);
    }
    markers.markers.push_back(path);

    for (std::size_t index = 0; index < mission_runtime_->trajectory().waypoints().size();
         ++index) {
      const auto &waypoint = mission_runtime_->trajectory().waypoints()[index];
      visualization_msgs::msg::Marker marker;
      marker.header = path.header;
      marker.ns = "mission_waypoints";
      marker.id = static_cast<int>(index + 1);
      marker.type = visualization_msgs::msg::Marker::SPHERE;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = waypoint.position[0];
      marker.pose.position.y = waypoint.position[1];
      marker.pose.position.z = waypoint.position[2];
      marker.scale.x = marker.scale.y = marker.scale.z = 0.4;
      if (index == mission_runtime_->trajectory().currentWaypointIndex() && mission_runtime_->trajectory().active()) {
        marker.color.r = 1.0F;
        marker.color.g = 0.6F;
      } else {
        marker.color.r = 0.1F;
        marker.color.g = 0.5F;
        marker.color.b = 1.0F;
      }
      marker.color.a = 0.8F;
      markers.markers.push_back(marker);
    }

    if (last_reference_ && !last_reference_->points.empty()) {
      visualization_msgs::msg::Marker preview;
      preview.header = path.header;
      preview.ns = "mpc_horizon_preview";
      preview.id = 100;
      preview.type = visualization_msgs::msg::Marker::LINE_STRIP;
      preview.action = visualization_msgs::msg::Marker::ADD;
      preview.scale.x = 0.08;
      preview.color.g = 1.0F;
      preview.color.b = 1.0F;
      preview.color.a = 0.9F;
      for (const auto &trajectory_point : last_reference_->points) {
        geometry_msgs::msg::Point point;
        point.x = trajectory_point.position[0];
        point.y = trajectory_point.position[1];
        point.z = trajectory_point.position[2];
        preview.points.push_back(point);
      }
      markers.markers.push_back(preview);
    }
    visualization_publisher_->publish(markers);
  }

  std::unique_ptr<mpc_controller::application::MissionRuntime>
      mission_runtime_;
  std::array<double, 3> hold_position_{0.0, 0.0, 1.0};
  std::string frame_id_{"map"};
  double hold_yaw_rad_ = 0.0;
  double acceptance_radius_m_ = 2.5;
  double acceptance_speed_m_s_ = 2.0;
  double speed_override_m_s_ = 0.0;
  double horizon_seconds_ = 3.0;
  double sample_period_seconds_ = 0.1;
  double planner_max_speed_xy_m_s_ = 30.0;
  double planner_max_speed_z_m_s_ = 3.0;
  double planner_max_acceleration_xy_m_s2_ = 6.0;
  double planner_max_acceleration_z_m_s2_ = 5.0;
  double planner_max_jerk_xy_m_s3_ = 8.0;
  double planner_max_jerk_z_m_s3_ = 4.0;
  double publish_rate_hz_ = 100.0;
  double state_timeout_seconds_ = 0.25;
  bool hold_reference_captured_ = false;
  bool pending_hold_capture_ = false;
  std::string active_controller_;
  bool last_state_control_ready_ = false;
  bool visualization_enabled_ = true;
  double visualization_publish_rate_hz_ = 20.0;
  bool valid_config_ = false;
  uint64_t trajectory_id_ = 1;
  std::optional<Reference> last_reference_;
  std::optional<SteadyClock::time_point> last_state_received_at_;
  std::optional<SteadyClock::time_point> last_visualization_published_at_;
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
      mission_completed_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      visualization_publisher_;
  rclcpp::Subscription<State>::SharedPtr state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      mpc_mode_state_subscription_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr
      px4_pid_mode_state_subscription_;
  rclcpp::Service<mpc_controller::srv::LoadAndStartMission>::SharedPtr
      load_and_start_mission_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_mission_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_mission_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  if (argc == 4 && std::string(argv[1]) == "--mission-start") {
    return runMissionClient(argv[2], argv[3]);
  }

  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReferenceGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
