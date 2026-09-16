#include "mpc_controller/mission/mission_json_parser.hpp"
#include "mpc_controller/mission/mission_trajectory.hpp"
#include "mpc_controller/msg/reference_trajectory.hpp"
#include "mpc_controller/msg/trajectory_point.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"

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
#include <vector>

class ReferenceGeneratorNode final : public rclcpp::Node {
public:
  ReferenceGeneratorNode() : Node("reference_generator_node") {
    declareAndGet("frame_id", frame_id_);
    declareAndGet("mission_file_path", mission_file_path_);
    declareAndGet("mission_acceptance_radius_m", acceptance_radius_m_);
    declareAndGet("mission_speed_override_m_s", speed_override_m_s_);
    declareAndGet("hold_yaw_rad", hold_yaw_rad_);
    declareAndGet("auto_capture_current_hold", auto_capture_current_hold_);
    declareAndGet("state_timeout_seconds", state_timeout_seconds_);
    declareAndGet("horizon_seconds", horizon_seconds_);
    declareAndGet("sample_period_seconds", sample_period_seconds_);
    declareAndGet("publish_rate_hz", publish_rate_hz_);
    declareAndGet("visualization_enabled", visualization_enabled_);
    declareAndGet("visualization_publish_rate_hz",
                  visualization_publish_rate_hz_);

    declare_parameter("hold_position", std::vector<double>{0.0, 0.0, 1.0});
    getVectorParameter("hold_position", hold_position_);

    valid_config_ = !frame_id_.empty() && positiveFinite(horizon_seconds_) &&
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
    start_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "~/start_mission",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          response->success = startMission();
          response->message = response->success
                                  ? "Mission started"
                                  : "Mission JSON could not be loaded";
        });
    reset_mission_service_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_mission",
        [this](const std_srvs::srv::Trigger::Request::SharedPtr,
               std_srvs::srv::Trigger::Response::SharedPtr response) {
          generator_->reset();
          response->success = true;
          response->message = "Mission reset to initial state";
        });

    loadMission();
    const auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0)));
    timer_ = create_wall_timer(
        period, std::bind(&ReferenceGeneratorNode::publish, this));
    RCLCPP_INFO(get_logger(),
                "Reference generator initialized: mission='%s' waypoints=%zu "
                "rate=%.1f Hz",
                mission_file_path_.c_str(), generator_->waypoints().size(),
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

  bool loadMission() {
    if (mission_file_path_.empty()) {
      RCLCPP_ERROR(get_logger(), "Mission file path is empty");
      return false;
    }
    const auto mission = mission_source_.load(mission_file_path_);
    std::string error;
    if (!generator_->setMission(mission, error)) {
      RCLCPP_ERROR(get_logger(), "Failed to load mission JSON '%s': %s",
                   mission_file_path_.c_str(), error.c_str());
      return false;
    }
    RCLCPP_INFO(get_logger(), "Loaded %zu mission waypoints from '%s'",
                generator_->waypoints().size(), mission_file_path_.c_str());
    return true;
  }

  bool startMission() {
    if (generator_->waypoints().empty() && !loadMission()) {
      return false;
    }
    if (!generator_->start(steadySeconds())) {
      return false;
    }
    const auto &target = generator_->waypoints().front();
    RCLCPP_INFO(
        get_logger(),
        "Mission started: %zu waypoints, target='%s' [%.2f, %.2f, %.2f]",
        generator_->waypoints().size(), target.id.c_str(), target.position[0],
        target.position[1], target.position[2]);
    return true;
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
    generator_->updateVehicleState(state);

    if (auto_capture_current_hold_ && !hold_reference_captured_ &&
        state.valid) {
      generator_->captureHoldFromVehicle();
      hold_reference_captured_ = true;
      RCLCPP_INFO(get_logger(),
                  "Initial hold captured: [%.3f, %.3f, %.3f], yaw=%.3f rad",
                  state.position[0], state.position[1], state.position[2],
                  state.yaw);
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
                  "Mission completed; handing off to native landing");
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

  mpc_controller::mission::MissionJsonParser mission_source_;
  std::unique_ptr<mpc_controller::mission::MissionReferenceGenerator>
      generator_;
  std::array<double, 3> hold_position_{0.0, 0.0, 1.0};
  std::string mission_file_path_{"config/missions/benchmark_square.json"};
  std::string frame_id_{"map"};
  double hold_yaw_rad_ = 0.0;
  double acceptance_radius_m_ = 2.5;
  double speed_override_m_s_ = 0.0;
  double horizon_seconds_ = 3.0;
  double sample_period_seconds_ = 0.1;
  double publish_rate_hz_ = 50.0;
  double state_timeout_seconds_ = 0.25;
  bool auto_capture_current_hold_ = true;
  bool hold_reference_captured_ = false;
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
