#include "mpc_controller/msg/force_attitude_setpoint.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/domain/mission/loader.hpp"
#include "mpc_controller/adapters/geometry_mapper.hpp"

#include <px4_msgs/msg/hover_thrust_estimate.hpp>
#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/experimental/attitude.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

class VehicleStateBridgeNode final : public rclcpp::Node
{
public:
  VehicleStateBridgeNode() : Node("vehicle_state_bridge_node")
  {
    declare_parameter("output_topic", output_topic_);
    declare_parameter("frame_id", frame_id_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("max_sample_skew_seconds", max_sample_skew_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("output_topic", output_topic_);
    get_parameter("frame_id", frame_id_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("max_sample_skew_seconds", max_sample_skew_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    position_subscription_ = create_subscription<Px4Position>("/fmu/out/vehicle_local_position_v1", qos,
      std::bind(&VehicleStateBridgeNode::positionCallback, this,std::placeholders::_1));
    attitude_subscription_ = create_subscription<Px4Attitude>("/fmu/out/vehicle_attitude", qos,
      std::bind(&VehicleStateBridgeNode::attitudeCallback, this,std::placeholders::_1));
    angular_velocity_subscription_ = create_subscription<Px4AngularVelocity>("/fmu/out/vehicle_angular_velocity", qos,
      std::bind(&VehicleStateBridgeNode::angularVelocityCallback, this,std::placeholders::_1));
    state_publisher_ = create_publisher<State>(output_topic_, 10);
    timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0))),
        std::bind(&VehicleStateBridgeNode::publish, this));
  }

private:
  using State = mpc_controller::msg::VehicleState;
  using Px4Position = px4_msgs::msg::VehicleLocalPosition;
  using Px4Attitude = px4_msgs::msg::VehicleAttitude;
  using Px4AngularVelocity = px4_msgs::msg::VehicleAngularVelocity;
  using Clock = std::chrono::steady_clock;

  struct Cache
  {
    Clock::time_point received_at{};
    uint64_t last_timestamp_sample = 0;
    uint64_t last_boot_timestamp_sample = 0;
    uint64_t last_synced_timestamp_sample = 0;
    bool last_timestamp_was_synced = false;
    bool received = false;
    uint64_t gap_count = 0;
    double gap_sum_seconds = 0.0;
    double gap_max_seconds = 0.0;
    uint64_t receipt_steady_timestamp_ns = 0;
  };

  static uint64_t steadyTimestampNs(const Clock::time_point time) noexcept
  {
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(
      time.time_since_epoch()).count());
  }

  static void recordReception(Cache &cache, Clock::time_point now)
  {
    if (cache.received) {
      const double gap = std::chrono::duration<double>(now - cache.received_at).count();
      if (std::isfinite(gap) && gap >= 0.0) {
        ++cache.gap_count;
        cache.gap_sum_seconds += gap;
        cache.gap_max_seconds = std::max(cache.gap_max_seconds, gap);
      }
    }
    cache.received_at = now;
    cache.receipt_steady_timestamp_ns = steadyTimestampNs(now);
    cache.received = true;
  }

  void positionCallback(const Px4Position::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!acceptTimestamp(position_cache_, message->timestamp_sample)) {
      return;
    }
    position_ = *message;
    recordReception(position_cache_, Clock::now());
  }

  void attitudeCallback(const Px4Attitude::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!acceptTimestamp(attitude_cache_, message->timestamp_sample)) {
      return;
    }
    attitude_ = *message;
    recordReception(attitude_cache_, Clock::now());
  }

  void angularVelocityCallback(const Px4AngularVelocity::SharedPtr message)
  {
    if (!message) {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    if (!acceptTimestamp(angular_velocity_cache_, message->timestamp_sample)) {
      return;
    }
    angular_velocity_ = *message;
    recordReception(angular_velocity_cache_, Clock::now());
  }

  static mpc_controller::state_check::Timing sourceTiming(
    const Cache &cache, uint64_t sample_timestamp, uint64_t evaluation_ns)
  {
    mpc_controller::state_check::Timing result;
    result.sample_time = sample_timestamp;
    result.received = cache.received;
    if (cache.received && evaluation_ns >= cache.receipt_steady_timestamp_ns) {
      result.age = static_cast<double>(evaluation_ns - cache.receipt_steady_timestamp_ns) * 1.0e-9;
    } else if (cache.received) {
      result.age = std::numeric_limits<double>::quiet_NaN();
    }
    return result;
  }

  static double sampleSkewMs(
    uint64_t position_timestamp, uint64_t attitude_timestamp,
    uint64_t angular_velocity_timestamp)
  {
    if (position_timestamp == 0 || attitude_timestamp == 0 || angular_velocity_timestamp == 0) {
      return std::numeric_limits<double>::infinity();
    }
    const auto minimum = std::min({position_timestamp, attitude_timestamp, angular_velocity_timestamp});
    const auto maximum = std::max({position_timestamp, attitude_timestamp, angular_velocity_timestamp});
    return static_cast<double>(maximum - minimum) * 1.0e-3;
  }

  bool acceptTimestamp(Cache &cache, uint64_t timestamp_sample)
  {
    if (timestamp_sample == 0U) {
      ++timestamp_rejection_count_;
      return false;
    }
    const bool synchronized = mpc_controller::frame::synchronizedTimestamp(timestamp_sample);
    uint64_t &history = synchronized ? cache.last_synced_timestamp_sample : cache.last_boot_timestamp_sample;
    if (mpc_controller::frame::timestampMonotonic(history, timestamp_sample)) {
      if (cache.received && synchronized != cache.last_timestamp_was_synced) {
        ++timestamp_domain_switch_count_;
      }
      history = timestamp_sample;
      cache.last_timestamp_sample = timestamp_sample;
      cache.last_timestamp_was_synced = synchronized;
      return true;
    }
    ++timestamp_rejection_count_;
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,"VehicleState input rejected: PX4 timestamp is zero");
    return false;
  }

  void publish()
  {
    Px4Position position;
    Px4Attitude attitude;
    Px4AngularVelocity angular_velocity;
    const auto now = Clock::now();
    const uint64_t evaluation_ns = steadyTimestampNs(now);
    mpc_controller::state_check::Timing position_timing;
    mpc_controller::state_check::Timing attitude_timing;
    mpc_controller::state_check::Timing angular_velocity_timing;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reportTiming(now);
      position = position_;
      attitude = attitude_;
      angular_velocity = angular_velocity_;
      position_timing = sourceTiming(position_cache_, position.timestamp_sample, evaluation_ns);
      attitude_timing = sourceTiming(attitude_cache_, attitude.timestamp_sample, evaluation_ns);
      angular_velocity_timing = sourceTiming(angular_velocity_cache_, angular_velocity.timestamp_sample, evaluation_ns);
    }

    const auto freshness_decision = mpc_controller::state_check::evaluate(
      position_timing, attitude_timing, angular_velocity_timing,
      state_timeout_seconds_, max_sample_skew_seconds_);
    const double current_sample_skew_ms = sampleSkewMs(
      position.timestamp_sample, attitude.timestamp_sample, angular_velocity.timestamp_sample);
    if (!freshness_decision.valid) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (freshness_decision.reason == mpc_controller::state_check::Reject::sample_skew) {
        ++skew_rejection_count_;
      } else {
        ++stale_rejection_count_;
      }
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
      "VehicleState not published: reason=%s pos_age=%.3f ms att_age=%.3f ms ang_age=%.3f ms sample_skew=%.3f ms",
        mpc_controller::state_check::reasonName(freshness_decision.reason),
        position_timing.age * 1.0e3, attitude_timing.age * 1.0e3,
        angular_velocity_timing.age * 1.0e3, current_sample_skew_ms);
      return;
    }

    mpc_controller::frame::Px4LocalPositionSample local_sample;
    local_sample.timestamp_sample = position.timestamp_sample;
    local_sample.xy_valid = position.xy_valid;
    local_sample.z_valid = position.z_valid;
    local_sample.v_xy_valid = position.v_xy_valid;
    local_sample.v_z_valid = position.v_z_valid;
    local_sample.heading_good_for_control = position.heading_good_for_control;
    local_sample.position_ned = {position.x, position.y, position.z};
    local_sample.velocity_ned = {position.vx, position.vy, position.vz};
    local_sample.acceleration_ned = {position.ax, position.ay, position.az};

    mpc_controller::frame::Px4AttitudeSample attitude_sample;
    attitude_sample.timestamp_sample = attitude.timestamp_sample;
    attitude_sample.body_frd_to_world_ned = {attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]};

    mpc_controller::frame::Px4AngularVelocitySample angular_sample;
    angular_sample.timestamp_sample = angular_velocity.timestamp_sample;
    angular_sample.body_rate_frd = {angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]};

    mpc_controller::frame::VehicleStateData converted;
    if (!mpc_controller::frame::convert(local_sample, attitude_sample, angular_sample, converted)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "VehicleState not published: PX4 estimator data invalid or non-finite");
      return;
    }

    State state;
    state.header.stamp = get_clock()->now();
    state.header.frame_id = frame_id_;
    state.position = converted.position_enu;
    state.velocity = converted.velocity_enu;
    state.acceleration = converted.acceleration_enu;
    state.attitude = converted.body_flu_to_world_enu;
    state.yaw = converted.yaw_enu;
    state.valid = converted.position_valid && converted.velocity_valid 
    && converted.acceleration_valid && converted.attitude_valid && converted.body_rate_valid;
    state.position_valid = converted.position_valid;
    state.velocity_valid = converted.velocity_valid;
    state.acceleration_valid = converted.acceleration_valid;
    state.attitude_valid = converted.attitude_valid;
    state.heading_valid = converted.heading_valid;
    state.control_ready = converted.control_ready;
    state_publisher_->publish(state);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++state_publish_count_;
    }
  }

  void reportTiming(Clock::time_point now)
  {
    if (std::chrono::duration<double>(now - timing_report_at_).count() < 5.0) {
      return;
    }
    timing_report_at_ = now;
    const auto age_ms = [now](const Cache &cache) {
      return cache.received ? std::chrono::duration<double>(now - cache.received_at).count() * 1.0e3 :
        std::numeric_limits<double>::infinity();
    };
    const auto mean_gap_ms = [](const Cache &cache) {
      return cache.gap_count > 0 ? cache.gap_sum_seconds / static_cast<double>(cache.gap_count) * 1.0e3 : 0.0;
    };
    const uint64_t minimum_sample = std::min({position_.timestamp_sample, attitude_.timestamp_sample,
      angular_velocity_.timestamp_sample});
    const uint64_t maximum_sample = std::max({position_.timestamp_sample, attitude_.timestamp_sample,
      angular_velocity_.timestamp_sample});
    const double skew_ms = maximum_sample >= minimum_sample ?
      static_cast<double>(maximum_sample - minimum_sample) * 1.0e-3 : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "VehicleState timing: published=%lu timestamp_reject=%lu clock_switch=%lu "
      "stale_reject=%lu skew_reject=%lu age_ms[pos att ang]=[%.1f %.1f %.1f] "
      "gap_mean_ms[pos att ang]=[%.2f %.2f %.2f] gap_max_ms[pos att ang]=[%.2f %.2f %.2f] sample_skew_ms=%.3f",
      static_cast<unsigned long>(state_publish_count_),
      static_cast<unsigned long>(timestamp_rejection_count_),
      static_cast<unsigned long>(timestamp_domain_switch_count_),
      static_cast<unsigned long>(stale_rejection_count_),
      static_cast<unsigned long>(skew_rejection_count_),
      age_ms(position_cache_), age_ms(attitude_cache_), age_ms(angular_velocity_cache_),
      mean_gap_ms(position_cache_), mean_gap_ms(attitude_cache_),
      mean_gap_ms(angular_velocity_cache_), position_cache_.gap_max_seconds * 1.0e3,
      attitude_cache_.gap_max_seconds * 1.0e3,
      angular_velocity_cache_.gap_max_seconds * 1.0e3, skew_ms);
  }

  std::mutex mutex_;
  Px4Position position_{};
  Px4Attitude attitude_{};
  Px4AngularVelocity angular_velocity_{};
  Cache position_cache_;
  Cache attitude_cache_;
  Cache angular_velocity_cache_;
  Clock::time_point timing_report_at_ = Clock::now();
  uint64_t state_publish_count_ = 0;
  uint64_t timestamp_rejection_count_ = 0;
  uint64_t timestamp_domain_switch_count_ = 0;
  uint64_t stale_rejection_count_ = 0;
  uint64_t skew_rejection_count_ = 0;
  std::string output_topic_ = "vehicle_state";
  std::string frame_id_ = "map";
  double state_timeout_seconds_ = 0.25;
  double max_sample_skew_seconds_ = 0.10;
  double publish_rate_hz_ = 100.0;
  rclcpp::Subscription<Px4Position>::SharedPtr position_subscription_;
  rclcpp::Subscription<Px4Attitude>::SharedPtr attitude_subscription_;
  rclcpp::Subscription<Px4AngularVelocity>::SharedPtr angular_velocity_subscription_;
  rclcpp::Publisher<State>::SharedPtr state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

static const std::string kNodeName = "px4_attitude_mode_node";

class MpcFlightMode : public px4_ros2::ModeBase {
public:
  explicit MpcFlightMode(rclcpp::Node &node)
      : ModeBase(node, Settings{"MPC Controller"}.preventArming(true)),  // mode can be activated while disarm
        node_(node) {
    attitude_setpoint_ = std::make_shared<px4_ros2::AttitudeSetpointType>(*this);

    force_setpoint_sub_ = node.create_subscription<mpc_controller::msg::ForceAttitudeSetpoint>("force_attitude_setpoint", rclcpp::QoS(10),
            [this](const mpc_controller::msg::ForceAttitudeSetpoint::SharedPtr msg) {
              if (!msg) {
                return;
              }
              std::lock_guard<std::mutex> lock(mutex_);
              latest_setpoint_ = *msg;
              last_setpoint_time_ = node_.now();
              sendSetpoint();
            });

    hover_thrust_sub_ = node.create_subscription<px4_msgs::msg::HoverThrustEstimate>("/fmu/out/hover_thrust_estimate", rclcpp::QoS(10).best_effort(),
            [this](const px4_msgs::msg::HoverThrustEstimate::SharedPtr msg) {
              if (msg && msg->valid && std::isfinite(msg->hover_thrust) && msg->hover_thrust > 0.05f) {
                std::lock_guard<std::mutex> lock(mutex_);
                hover_thrust_ = msg->hover_thrust;
              }
            });

    mission_completed_sub_ = node.create_subscription<std_msgs::msg::Bool>("/reference_generator_node/mission_completed", rclcpp::QoS(10),
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
          if (msg && msg->data && isActive()) {
            RCLCPP_INFO(
                node_.get_logger(),
                "Mission completed signal received; completing External Mode");
            completed(px4_ros2::Result::Success);
          }
        });
    external_mode_state_publisher_ = node.create_publisher<std_msgs::msg::Bool>(mpc_controller::mission::kMpcExternalModeStateTopic.data(),
        rclcpp::QoS(1).reliable().transient_local());
    publishModeState(false);

    RCLCPP_INFO(node_.get_logger(),
        "MpcFlightMode initialized: registered with PX4 Flight Mode Manager");
  }

  void onActivate() override {
    publishModeState(true);
    RCLCPP_INFO(node_.get_logger(), "MpcFlightMode ACTIVATED");
  }

  void onDeactivate() override {
    publishModeState(false);
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
    const Eigen::Vector3f thrust_frd(0.f, 0.f,-static_cast<float>(thrust_norm));

    const auto yaw_rate_ned = mpc_controller::px4_control::enuYawRateToNed(latest_setpoint_->desired_yaw_rate_rad_s);
    if (!yaw_rate_ned) {
      RCLCPP_WARN_THROTTLE(node_.get_logger(), *node_.get_clock(), 2000,"Rejecting non-finite ENU yaw-rate setpoint");
      return;
    }

    attitude_setpoint_->update(q_frd_ned, thrust_frd,static_cast<float>(*yaw_rate_ned));
  }

private:
  void publishModeState(bool active) {
    std_msgs::msg::Bool message;
    message.data = active;
    external_mode_state_publisher_->publish(message);
  }

  rclcpp::Node &node_;
  std::shared_ptr<px4_ros2::AttitudeSetpointType> attitude_setpoint_;
  rclcpp::Subscription<mpc_controller::msg::ForceAttitudeSetpoint>::SharedPtr force_setpoint_sub_;
  rclcpp::Subscription<px4_msgs::msg::HoverThrustEstimate>::SharedPtr hover_thrust_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr mission_completed_sub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr external_mode_state_publisher_;
  std::optional<mpc_controller::msg::ForceAttitudeSetpoint> latest_setpoint_;
  std::optional<rclcpp::Time> last_setpoint_time_;
  double hover_thrust_ = 0.60;
  std::mutex mutex_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    auto attitude_node = std::make_shared<px4_ros2::NodeWithMode<MpcFlightMode>>(kNodeName, false);
    auto state_node = std::make_shared<VehicleStateBridgeNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(attitude_node);
    executor.add_node(state_node);
    executor.spin();
  } catch (const std::exception &e) {
    std::cerr << "px4_attitude_mode_node exception: " << e.what() << std::endl;
  }
  rclcpp::shutdown();
  return 0;
}
