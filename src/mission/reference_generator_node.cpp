#include "mpc_controller/mission/controller_profile.hpp"
#include "mpc_controller/mission/mission_trajectory.hpp"
#include "mpc_controller/mission/runtime_mission_loader.hpp"
#include "mpc_controller/msg/mpc_mission_plan.hpp"
#include "mpc_controller/msg/mpc_mission_waypoint.hpp"
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
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

class ReferenceGeneratorNode final : public rclcpp::Node {
public:
  ReferenceGeneratorNode() : Node("reference_generator_node") {
    declareAndGet("frame_id", frame_id_);
    declareAndGet("mission_acceptance_radius_m", acceptance_radius_m_);
    declareAndGet("mission_speed_override_m_s", speed_override_m_s_);
    declareAndGet("hold_yaw_rad", hold_yaw_rad_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("horizon_seconds", horizon_seconds_);
    declareAndGet("sample_period_seconds", sample_period_seconds_);
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("visualization_enabled", visualization_enabled_);
    declareAndGet("visualization_publish_rate_hz",
                  visualization_publish_rate_hz_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    getVectorParameter("hold_position", hold_position_);

    valid_config_ =
        !frame_id_.empty() && positiveFinite(acceptance_radius_m_) &&
        std::isfinite(speed_override_m_s_) && speed_override_m_s_ >= 0.0 &&
        positiveFinite(horizon_seconds_) &&
        positiveFinite(sample_period_seconds_) &&
        positiveFinite(publish_rate_hz_) &&
        positiveFinite(state_timeout_seconds_) &&
        horizon_seconds_ >= sample_period_seconds_;
    if (!valid_config_) {
      RCLCPP_ERROR(
          get_logger(),
          "Invalid reference generator parameters; publishing disabled");
      return;
    }

    mpc_controller::mission::MissionReferenceGenerator::Config config;
    config.reference.hold_position = hold_position_;
    config.reference.hold_yaw_rad = hold_yaw_rad_;
    config.acceptance_radius_m = acceptance_radius_m_;
    config.speed_override_m_s = speed_override_m_s_;
    config.horizon_seconds = horizon_seconds_;
    config.sample_period_seconds = sample_period_seconds_;
    generator_ =
        std::make_unique<mpc_controller::mission::MissionReferenceGenerator>(
            config);
    runtime_mission_loader_ =
        std::make_unique<mpc_controller::mission::RuntimeMissionLoader>(
            *generator_);

    publisher_ = create_publisher<Reference>("reference_trajectory", 10);
    mpc_plan_publisher_ = create_publisher<MpcPlan>(
        "mpc_mission_plan", rclcpp::QoS(1).reliable().transient_local());
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
          if (active_controller_ == mpc_controller::mission::kMpcController) {
            publishMpcPlan(false);
          }
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
  using MpcPlan = mpc_controller::msg::MpcMissionPlan;
  using MpcWaypoint = mpc_controller::msg::MpcMissionWaypoint;
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

    if (controller == mpc_controller::mission::kMpcController) {
      publishMpcPlan(false);
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
        runtime_mission_loader_->loadAndStart(mission_path, steadySeconds());
    if (!result.success) {
      transitionToMeasuredHold("Mission request rejected");
      RCLCPP_ERROR(get_logger(), "Mission request rejected: %s",
                   result.message.c_str());
      return result;
    }
    const auto &target = generator_->waypoints().front();
    if (controller == mpc_controller::mission::kMpcController) {
      publishMpcPlan(true);
      generator_->reset();
    }
    RCLCPP_INFO(
        get_logger(),
        "Mission started: %zu waypoints, target='%s' [%.2f, %.2f, %.2f]",
        generator_->waypoints().size(), target.id.c_str(), target.position[0],
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
    if (controller == mpc_controller::mission::kMpcController) {
      publishMpcPlan(false);
    }
    active_controller_.clear();
    transitionToMeasuredHold("External Mode deactivation");
  }

  void transitionToMeasuredHold(const char *reason) {
    runtime_mission_loader_->abortToHold();
    pending_hold_capture_ = true;
    hold_reference_captured_ = false;
    captureHoldIfFresh();
    RCLCPP_INFO(get_logger(), "%s; waiting for measured-position hold", reason);
  }

  static uint8_t mpcItemType(mpc_controller::mission::ItemType type) {
    switch (type) {
    case mpc_controller::mission::ItemType::Takeoff:
      return MpcWaypoint::TAKEOFF;
    case mpc_controller::mission::ItemType::Land:
      return MpcWaypoint::LAND;
    case mpc_controller::mission::ItemType::Rtl:
      return MpcWaypoint::RTL;
    default:
      return MpcWaypoint::WAYPOINT;
    }
  }

  void publishMpcPlan(bool active) {
    if (!mpc_plan_publisher_) {
      return;
    }
    MpcPlan plan;
    plan.header.stamp = get_clock()->now();
    plan.header.frame_id = frame_id_;
    plan.mission_id = mpc_plan_id_++;
    plan.active = active;
    if (active && generator_) {
      const auto &waypoints = generator_->waypoints();
      plan.waypoints.reserve(waypoints.size());
      for (const auto &source : waypoints) {
        MpcWaypoint target;
        target.id = source.id;
        target.item_type = mpcItemType(source.type);
        target.position = source.position;
        target.acceptance_radius_m =
            source.type == mpc_controller::mission::ItemType::Land
                ? 0.35
                : acceptance_radius_m_;
        target.hold_duration_s = source.hold_duration_s;
        target.horizontal_speed_m_s = source.horizontal_speed;
        target.vertical_speed_m_s = source.vertical_speed;
        target.heading_rad = source.heading_rad;
        target.max_heading_rate_rad_s = source.max_heading_rate_rad_s;
        target.maximum_acceleration_m_s2 = source.maximum_acceleration_m_s2;
        target.maximum_jerk_m_s3 = source.maximum_jerk_m_s3;
        plan.waypoints.push_back(std::move(target));
      }
    }
    mpc_plan_publisher_->publish(plan);
  }

  void captureHoldIfFresh() {
    if (!pending_hold_capture_ || !hasFreshControlReadyState()) {
      return;
    }
    generator_->captureHoldFromVehicle();
    pending_hold_capture_ = false;
    hold_reference_captured_ = true;
    RCLCPP_INFO(get_logger(), "Measured hold captured");
  }

  void stateCallback(const State::SharedPtr message) {
    last_state_received_at_ = SteadyClock::now();
    mpc_controller::mission::MissionReferenceGenerator::VehicleState state;
    state.position = {message->position[0], message->position[1],
                      message->position[2]};
    state.velocity = {message->velocity[0], message->velocity[1],
                      message->velocity[2]};
    state.yaw = message->yaw;
    state.valid = message->control_ready;
    last_state_control_ready_ = state.valid;
    generator_->updateVehicleState(state);

    // Before the PX4 External Mode is selected, continuously follow the
    // admitted vehicle pose. Activation freezes that already-current target.
    if (state.valid && active_controller_.empty()) {
      generator_->captureHoldFromVehicle();
      hold_reference_captured_ = true;
    } else if (state.valid && pending_hold_capture_) {
      captureHoldIfFresh();
    }
  }

  void publish() {
    if (!valid_config_ || !publisher_ || !generator_) {
      return;
    }
    const auto steady_now = SteadyClock::now();
    if (!last_state_received_at_ ||
        std::chrono::duration<double>(steady_now - *last_state_received_at_)
                .count() > state_timeout_seconds_) {
      generator_->invalidateVehicleState();
    }
    const auto update = generator_->update(
        std::chrono::duration<double>(steady_now.time_since_epoch()).count());
    if (update.waypoint_reached) {
      const auto &waypoint =
          generator_->waypoints()[update.reached_waypoint_index];
      RCLCPP_INFO(
          get_logger(),
          "Waypoint %zu/%zu reached: id='%s', distance=%.2fm, crossed_plane=%s",
          update.reached_waypoint_index + 1, generator_->waypoints().size(),
          waypoint.id.c_str(), update.distance_to_waypoint_m,
          update.crossed_finish_plane ? "yes" : "no");
    }
    if (update.mission_completed) {
      std_msgs::msg::Bool completed;
      completed.data = true;
      mission_completed_publisher_->publish(completed);
      RCLCPP_INFO(get_logger(),
                  "Mission completed; holding the final mission position");
    }

    Reference message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.trajectory_id = trajectory_id_++;
    message.hold_after_end = true;
    message.points.reserve(update.samples.size());
    const uint64_t sample_nanoseconds =
        static_cast<uint64_t>(sample_period_seconds_ * 1.0e9);
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
    for (const auto &waypoint : generator_->waypoints()) {
      geometry_msgs::msg::Point point;
      point.x = waypoint.position[0];
      point.y = waypoint.position[1];
      point.z = waypoint.position[2];
      path.points.push_back(point);
    }
    markers.markers.push_back(path);

    for (std::size_t index = 0; index < generator_->waypoints().size();
         ++index) {
      const auto &waypoint = generator_->waypoints()[index];
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
      if (index == generator_->currentWaypointIndex() && generator_->active()) {
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

  std::unique_ptr<mpc_controller::mission::MissionReferenceGenerator>
      generator_;
  std::unique_ptr<mpc_controller::mission::RuntimeMissionLoader>
      runtime_mission_loader_;
  std::array<double, 3> hold_position_{0.0, 0.0, 1.0};
  std::string frame_id_{"map"};
  double hold_yaw_rad_ = 0.0;
  double acceptance_radius_m_ = 2.5;
  double speed_override_m_s_ = 0.0;
  double horizon_seconds_ = 3.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 50.0;
  double state_timeout_seconds_ = 0.25;
  bool hold_reference_captured_ = false;
  bool pending_hold_capture_ = false;
  std::string active_controller_;
  bool last_state_control_ready_ = false;
  bool visualization_enabled_ = true;
  double visualization_publish_rate_hz_ = 20.0;
  bool valid_config_ = false;
  uint64_t trajectory_id_ = 1;
  uint64_t mpc_plan_id_ = 1;
  std::optional<Reference> last_reference_;
  std::optional<SteadyClock::time_point> last_state_received_at_;
  std::optional<SteadyClock::time_point> last_visualization_published_at_;
  rclcpp::Publisher<Reference>::SharedPtr publisher_;
  rclcpp::Publisher<MpcPlan>::SharedPtr mpc_plan_publisher_;
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
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ReferenceGeneratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
