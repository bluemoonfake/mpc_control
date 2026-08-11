#include "mpc_controller/frame_contract.hpp"
#include "mpc_controller/msg/vehicle_state_bridge_diagnostics.hpp"
#include "mpc_controller/msg/vehicle_state.hpp"
#include "mpc_controller/vehicle_state_bridge_diagnostics.hpp"

#include <px4_msgs/msg/vehicle_angular_velocity.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <limits>
#include <mutex>
#include <optional>

class VehicleStateBridgeNode final : public rclcpp::Node
{
public:
  VehicleStateBridgeNode()
  : Node("vehicle_state_bridge_node")
  {
    declare_parameter("position_topic", position_topic_);
    declare_parameter("attitude_topic", attitude_topic_);
    declare_parameter("angular_velocity_topic", angular_velocity_topic_);
    declare_parameter("output_topic", output_topic_);
    declare_parameter("frame_id", frame_id_);
    declare_parameter("state_timeout_seconds", state_timeout_seconds_);
    declare_parameter("max_sample_skew_seconds", max_sample_skew_seconds_);
    declare_parameter("publish_rate_hz", publish_rate_hz_);
    get_parameter("position_topic", position_topic_);
    get_parameter("attitude_topic", attitude_topic_);
    get_parameter("angular_velocity_topic", angular_velocity_topic_);
    get_parameter("output_topic", output_topic_);
    get_parameter("frame_id", frame_id_);
    get_parameter("state_timeout_seconds", state_timeout_seconds_);
    get_parameter("max_sample_skew_seconds", max_sample_skew_seconds_);
    get_parameter("publish_rate_hz", publish_rate_hz_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    position_subscription_ = create_subscription<Px4Position>(
      position_topic_, qos, std::bind(&VehicleStateBridgeNode::positionCallback, this, std::placeholders::_1));
    attitude_subscription_ = create_subscription<Px4Attitude>(
      attitude_topic_, qos, std::bind(&VehicleStateBridgeNode::attitudeCallback, this, std::placeholders::_1));
    angular_velocity_subscription_ = create_subscription<Px4AngularVelocity>(
      angular_velocity_topic_, qos,
      std::bind(&VehicleStateBridgeNode::angularVelocityCallback, this, std::placeholders::_1));
    state_publisher_ = create_publisher<State>(output_topic_, 10);
    diagnostics_publisher_ = create_publisher<Diagnostics>("vehicle_state_bridge_diagnostics", 10);
    reset_diagnostics_service_ = create_service<std_srvs::srv::Trigger>(
      "~/reset_diagnostics",
      std::bind(
        &VehicleStateBridgeNode::resetDiagnostics, this, std::placeholders::_1,
        std::placeholders::_2));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::duration<double>(1.0 / std::max(publish_rate_hz_, 1.0))),
      std::bind(&VehicleStateBridgeNode::publish, this));
  }

private:
  using State = mpc_controller::msg::VehicleState;
  using Diagnostics = mpc_controller::msg::VehicleStateBridgeDiagnostics;
  using Px4Position = px4_msgs::msg::VehicleLocalPosition;
  using Px4Attitude = px4_msgs::msg::VehicleAttitude;
  using Px4AngularVelocity = px4_msgs::msg::VehicleAngularVelocity;
  using Clock = std::chrono::steady_clock;

  struct Cache
  {
    Clock::time_point received_at{};
    uint64_t last_timestamp_sample = 0;
    bool received = false;
    uint64_t accepted_count = 0;
    uint64_t gap_count = 0;
    double gap_sum_seconds = 0.0;
    double gap_max_seconds = 0.0;
    double last_interarrival_seconds = std::numeric_limits<double>::infinity();
    uint64_t receipt_steady_timestamp_ns = 0;
  };

  struct FirstReject
  {
    bool latched = false;
    uint64_t steady_timestamp_ns = 0;
    mpc_controller::vehicle_state_diagnostics::RejectReason reason =
      mpc_controller::vehicle_state_diagnostics::RejectReason::none;
    mpc_controller::vehicle_state_diagnostics::SourceTiming position;
    mpc_controller::vehicle_state_diagnostics::SourceTiming attitude;
    mpc_controller::vehicle_state_diagnostics::SourceTiming angular_velocity;
    double sample_skew_ms = std::numeric_limits<double>::infinity();
    uint64_t previous_vehicle_state_publication_steady_timestamp_ns = 0;
  };

  static uint64_t steadyTimestampNs(const Clock::time_point time) noexcept
  {
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(time.time_since_epoch()).count());
  }

  static void recordReception(Cache &cache, Clock::time_point now)
  {
    if (cache.received) {
      const double gap = std::chrono::duration<double>(now - cache.received_at).count();
      if (std::isfinite(gap) && gap >= 0.0) {
        ++cache.gap_count;
        cache.gap_sum_seconds += gap;
        cache.gap_max_seconds = std::max(cache.gap_max_seconds, gap);
        cache.last_interarrival_seconds = gap;
      }
    }
    cache.received_at = now;
    cache.receipt_steady_timestamp_ns = steadyTimestampNs(now);
    cache.received = true;
    ++cache.accepted_count;
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

  bool fresh(const Cache &cache, Clock::time_point now) const noexcept
  {
    if (!cache.received) {
      return false;
    }
    const double age = std::chrono::duration<double>(now - cache.received_at).count();
    return mpc_controller::frame::sampleAgeValid(age, state_timeout_seconds_);
  }

  static mpc_controller::vehicle_state_diagnostics::SourceTiming sourceTiming(
    const Cache &cache, const uint64_t sample_timestamp, const uint64_t evaluation_ns)
  {
    mpc_controller::vehicle_state_diagnostics::SourceTiming result;
    result.sample_timestamp = sample_timestamp;
    result.receipt_steady_timestamp_ns = cache.receipt_steady_timestamp_ns;
    result.interarrival_seconds = cache.last_interarrival_seconds;
    result.received = cache.received;
    if (cache.received && evaluation_ns >= cache.receipt_steady_timestamp_ns) {
      result.age_seconds = static_cast<double>(evaluation_ns - cache.receipt_steady_timestamp_ns) * 1.0e-9;
    } else if (cache.received) {
      result.age_seconds = std::numeric_limits<double>::quiet_NaN();
    }
    return result;
  }

  static double sampleSkewMs(
    const uint64_t position_timestamp, const uint64_t attitude_timestamp,
    const uint64_t angular_velocity_timestamp)
  {
    if (position_timestamp == 0 || attitude_timestamp == 0 || angular_velocity_timestamp == 0) {
      return std::numeric_limits<double>::infinity();
    }
    const auto minimum = std::min({position_timestamp, attitude_timestamp, angular_velocity_timestamp});
    const auto maximum = std::max({position_timestamp, attitude_timestamp, angular_velocity_timestamp});
    return static_cast<double>(maximum - minimum) * 1.0e-3;
  }

  void latchFirstReject(
    const mpc_controller::vehicle_state_diagnostics::RejectReason reason,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &position,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &attitude,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &angular_velocity,
    const double sample_skew_ms, const uint64_t evaluation_ns)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (first_reject_.latched) {
      return;
    }
    first_reject_.latched = true;
    first_reject_.steady_timestamp_ns = evaluation_ns;
    first_reject_.reason = reason;
    first_reject_.position = position;
    first_reject_.attitude = attitude;
    first_reject_.angular_velocity = angular_velocity;
    first_reject_.sample_skew_ms = sample_skew_ms;
    first_reject_.previous_vehicle_state_publication_steady_timestamp_ns =
      last_vehicle_state_publication_steady_timestamp_ns_;
    RCLCPP_ERROR(
      get_logger(),
      "VehicleState first reject: reason=%s pos_age=%.3f ms att_age=%.3f ms "
      "ang_age=%.3f ms skew=%.3f ms",
      mpc_controller::vehicle_state_diagnostics::rejectReasonName(reason),
      position.age_seconds * 1.0e3, attitude.age_seconds * 1.0e3,
      angular_velocity.age_seconds * 1.0e3, sample_skew_ms);
  }

  void publishDiagnostics(
    const Clock::time_point now,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &position,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &attitude,
    const mpc_controller::vehicle_state_diagnostics::SourceTiming &angular_velocity,
    const double sample_skew_ms,
    const mpc_controller::vehicle_state_diagnostics::RejectReason reason,
    const bool state_published)
  {
    Diagnostics message;
    message.header.stamp = get_clock()->now();
    message.header.frame_id = frame_id_;
    message.sequence = ++diagnostics_sequence_;
    message.vehicle_state_published = state_published;
    message.publication_rejected = !state_published;
    message.reject_reason = static_cast<uint8_t>(reason);
    message.sample_skew_ms = sample_skew_ms;
    message.stale_position = !mpc_controller::vehicle_state_diagnostics::fresh(
      position, state_timeout_seconds_);
    message.stale_attitude = !mpc_controller::vehicle_state_diagnostics::fresh(
      attitude, state_timeout_seconds_);
    message.stale_angular_velocity = !mpc_controller::vehicle_state_diagnostics::fresh(
      angular_velocity, state_timeout_seconds_);
    message.position_sample_timestamp = position.sample_timestamp;
    message.attitude_sample_timestamp = attitude.sample_timestamp;
    message.angular_velocity_sample_timestamp = angular_velocity.sample_timestamp;
    message.position_receipt_steady_timestamp_ns = position.receipt_steady_timestamp_ns;
    message.attitude_receipt_steady_timestamp_ns = attitude.receipt_steady_timestamp_ns;
    message.angular_velocity_receipt_steady_timestamp_ns = angular_velocity.receipt_steady_timestamp_ns;
    message.position_age_ms = position.age_seconds * 1.0e3;
    message.attitude_age_ms = attitude.age_seconds * 1.0e3;
    message.angular_velocity_age_ms = angular_velocity.age_seconds * 1.0e3;
    message.position_interarrival_ms = position.interarrival_seconds * 1.0e3;
    message.attitude_interarrival_ms = attitude.interarrival_seconds * 1.0e3;
    message.angular_velocity_interarrival_ms = angular_velocity.interarrival_seconds * 1.0e3;
    FirstReject first_reject;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      message.position_accepted_count = position_cache_.accepted_count;
      message.attitude_accepted_count = attitude_cache_.accepted_count;
      message.angular_velocity_accepted_count = angular_velocity_cache_.accepted_count;
      message.stale_reject_count = stale_rejection_count_;
      message.skew_reject_count = skew_rejection_count_;
      message.last_vehicle_state_publication_steady_timestamp_ns =
        last_vehicle_state_publication_steady_timestamp_ns_;
      first_reject = first_reject_;
    }
    message.first_reject_latched = first_reject.latched;
    message.first_reject_steady_timestamp_ns = first_reject.steady_timestamp_ns;
    message.first_reject_reason = static_cast<uint8_t>(first_reject.reason);
    message.first_reject_position_age_ms = first_reject.position.age_seconds * 1.0e3;
    message.first_reject_attitude_age_ms = first_reject.attitude.age_seconds * 1.0e3;
    message.first_reject_angular_velocity_age_ms = first_reject.angular_velocity.age_seconds * 1.0e3;
    message.first_reject_sample_skew_ms = first_reject.sample_skew_ms;
    message.first_reject_position_sample_timestamp = first_reject.position.sample_timestamp;
    message.first_reject_attitude_sample_timestamp = first_reject.attitude.sample_timestamp;
    message.first_reject_angular_velocity_sample_timestamp = first_reject.angular_velocity.sample_timestamp;
    message.first_reject_position_receipt_steady_timestamp_ns =
      first_reject.position.receipt_steady_timestamp_ns;
    message.first_reject_attitude_receipt_steady_timestamp_ns =
      first_reject.attitude.receipt_steady_timestamp_ns;
    message.first_reject_angular_velocity_receipt_steady_timestamp_ns =
      first_reject.angular_velocity.receipt_steady_timestamp_ns;
    message.first_reject_previous_vehicle_state_publication_steady_timestamp_ns =
      first_reject.previous_vehicle_state_publication_steady_timestamp_ns;
    diagnostics_publisher_->publish(message);
    (void)now;
  }

  bool acceptTimestamp(Cache &cache, uint64_t timestamp_sample)
  {
    if (!mpc_controller::frame::timestampMonotonic(cache.last_timestamp_sample, timestamp_sample)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "VehicleState input rejected: PX4 timestamp is zero or moved backwards");
      return false;
    }
    cache.last_timestamp_sample = timestamp_sample;
    return true;
  }

  void publish()
  {
    Px4Position position;
    Px4Attitude attitude;
    Px4AngularVelocity angular_velocity;
    const auto now = Clock::now();
    const uint64_t evaluation_ns = steadyTimestampNs(now);
    mpc_controller::vehicle_state_diagnostics::SourceTiming position_timing;
    mpc_controller::vehicle_state_diagnostics::SourceTiming attitude_timing;
    mpc_controller::vehicle_state_diagnostics::SourceTiming angular_velocity_timing;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      reportTiming(now);
      position = position_;
      attitude = attitude_;
      angular_velocity = angular_velocity_;
      position_timing = sourceTiming(position_cache_, position.timestamp_sample, evaluation_ns);
      attitude_timing = sourceTiming(attitude_cache_, attitude.timestamp_sample, evaluation_ns);
      angular_velocity_timing = sourceTiming(
        angular_velocity_cache_, angular_velocity.timestamp_sample, evaluation_ns);
    }

    const auto freshness_decision = mpc_controller::vehicle_state_diagnostics::evaluate(
      position_timing, attitude_timing, angular_velocity_timing, evaluation_ns,
      state_timeout_seconds_, max_sample_skew_seconds_);
    const double current_sample_skew_ms = sampleSkewMs(
      position.timestamp_sample, attitude.timestamp_sample, angular_velocity.timestamp_sample);
    if (!freshness_decision.valid) {
      if (freshness_decision.reason ==
        mpc_controller::vehicle_state_diagnostics::RejectReason::sample_skew)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ++skew_rejection_count_;
      } else {
        std::lock_guard<std::mutex> lock(mutex_);
        ++stale_rejection_count_;
      }
      latchFirstReject(
        freshness_decision.reason, position_timing, attitude_timing, angular_velocity_timing,
        current_sample_skew_ms, evaluation_ns);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "VehicleState not published: reason=%s pos_age=%.3f ms att_age=%.3f ms "
        "ang_age=%.3f ms sample_skew=%.3f ms",
        mpc_controller::vehicle_state_diagnostics::rejectReasonName(freshness_decision.reason),
        position_timing.age_seconds * 1.0e3, attitude_timing.age_seconds * 1.0e3,
        angular_velocity_timing.age_seconds * 1.0e3, current_sample_skew_ms);
      publishDiagnostics(
        now, position_timing, attitude_timing, angular_velocity_timing,
        current_sample_skew_ms, freshness_decision.reason, false);
      return;
    }

    mpc_controller::frame::Px4LocalPositionSample local_sample;
    local_sample.timestamp_sample = position.timestamp_sample;
    local_sample.xy_valid = position.xy_valid;
    local_sample.z_valid = position.z_valid;
    local_sample.v_xy_valid = position.v_xy_valid;
    local_sample.v_z_valid = position.v_z_valid;
    local_sample.heading_good_for_control = position.heading_good_for_control;
    local_sample.xy_reset_counter = position.xy_reset_counter;
    local_sample.z_reset_counter = position.z_reset_counter;
    local_sample.heading_reset_counter = position.heading_reset_counter;
    local_sample.position_ned = {position.x, position.y, position.z};
    local_sample.velocity_ned = {position.vx, position.vy, position.vz};
    local_sample.acceleration_ned = {position.ax, position.ay, position.az};

    mpc_controller::frame::Px4AttitudeSample attitude_sample;
    attitude_sample.timestamp_sample = attitude.timestamp_sample;
    attitude_sample.quat_reset_counter = attitude.quat_reset_counter;
    attitude_sample.body_frd_to_world_ned = {
      attitude.q[0], attitude.q[1], attitude.q[2], attitude.q[3]};

    mpc_controller::frame::Px4AngularVelocitySample angular_sample;
    angular_sample.timestamp_sample = angular_velocity.timestamp_sample;
    angular_sample.body_rate_frd = {
      angular_velocity.xyz[0], angular_velocity.xyz[1], angular_velocity.xyz[2]};

    mpc_controller::frame::VehicleStateData converted;
    if (!mpc_controller::frame::convert(local_sample, attitude_sample, angular_sample, converted)) {
      latchFirstReject(
        mpc_controller::vehicle_state_diagnostics::RejectReason::non_finite,
        position_timing, attitude_timing, angular_velocity_timing,
        current_sample_skew_ms, evaluation_ns);
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "VehicleState not published: PX4 estimator data invalid or non-finite");
      publishDiagnostics(
        now, position_timing, attitude_timing, angular_velocity_timing,
        current_sample_skew_ms,
        mpc_controller::vehicle_state_diagnostics::RejectReason::non_finite, false);
      return;
    }

    State state;
    state.header.stamp = get_clock()->now();
    state.header.frame_id = frame_id_;
    state.position = converted.position_enu;
    state.velocity = converted.velocity_enu;
    state.acceleration = converted.acceleration_enu;
    state.attitude = converted.body_flu_to_world_enu;
    state.body_rate = converted.body_rate_flu;
    state.yaw = converted.yaw_enu;
    state.yaw_rate = converted.yaw_rate_enu;
    // `valid` means the measured state is usable by the controller
    // processing. Active attitude/thrust control additionally requires
    // PX4's heading readiness, exposed separately below.
    state.valid = converted.position_valid && converted.velocity_valid
      && converted.acceleration_valid && converted.attitude_valid
      && converted.body_rate_valid;
    state.position_valid = converted.position_valid;
    state.velocity_valid = converted.velocity_valid;
    state.acceleration_valid = converted.acceleration_valid;
    state.attitude_valid = converted.attitude_valid;
    state.body_rate_valid = converted.body_rate_valid;
    state.heading_valid = converted.heading_valid;
    state.yaw_valid = converted.heading_valid;
    state.control_ready = converted.control_ready;
    state.reset_counter = converted.reset_counter;
    state.reset_counter_valid = true;
    state_publisher_->publish(state);
    {
      std::lock_guard<std::mutex> lock(mutex_);
      ++state_publish_count_;
      last_vehicle_state_publication_steady_timestamp_ns_ = evaluation_ns;
    }
    publishDiagnostics(
      now, position_timing, attitude_timing, angular_velocity_timing,
      current_sample_skew_ms,
      mpc_controller::vehicle_state_diagnostics::RejectReason::none, true);
  }

  void resetDiagnostics(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    first_reject_ = FirstReject{};
    response->success = true;
    response->message = "VehicleState bridge first-reject diagnostic latch reset";
  }

  void reportTiming(Clock::time_point now)
  {
    if (std::chrono::duration<double>(now - timing_report_at_).count() < 5.0) {
      return;
    }
    timing_report_at_ = now;
    const auto age_ms = [now](const Cache &cache) {
        return cache.received
          ? std::chrono::duration<double>(now - cache.received_at).count() * 1.0e3
          : std::numeric_limits<double>::infinity();
      };
    const auto mean_gap_ms = [](const Cache &cache) {
        return cache.gap_count > 0
          ? cache.gap_sum_seconds / static_cast<double>(cache.gap_count) * 1.0e3
          : 0.0;
      };
    const uint64_t minimum_sample = std::min({
      position_.timestamp_sample, attitude_.timestamp_sample,
      angular_velocity_.timestamp_sample});
    const uint64_t maximum_sample = std::max({
      position_.timestamp_sample, attitude_.timestamp_sample,
      angular_velocity_.timestamp_sample});
    const double skew_ms = maximum_sample >= minimum_sample
      ? static_cast<double>(maximum_sample - minimum_sample) * 1.0e-3 : 0.0;
    RCLCPP_INFO(
      get_logger(),
      "VehicleState timing: published=%lu stale_reject=%lu skew_reject=%lu "
      "age_ms[pos att ang]=[%.1f %.1f %.1f] gap_mean_ms[pos att ang]=[%.2f %.2f %.2f] "
      "gap_max_ms[pos att ang]=[%.2f %.2f %.2f] sample_skew_ms=%.3f",
      static_cast<unsigned long>(state_publish_count_),
      static_cast<unsigned long>(stale_rejection_count_),
      static_cast<unsigned long>(skew_rejection_count_),
      age_ms(position_cache_), age_ms(attitude_cache_), age_ms(angular_velocity_cache_),
      mean_gap_ms(position_cache_), mean_gap_ms(attitude_cache_),
      mean_gap_ms(angular_velocity_cache_),
      position_cache_.gap_max_seconds * 1.0e3,
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
  uint64_t stale_rejection_count_ = 0;
  uint64_t skew_rejection_count_ = 0;
  uint64_t diagnostics_sequence_ = 0;
  uint64_t last_vehicle_state_publication_steady_timestamp_ns_ = 0;
  FirstReject first_reject_{};
  // PX4 v1.17 publishes this versioned uORB message on the DDS topic
  // vehicle_local_position_v1. Keep the topic configurable for other
  // px4_msgs revisions, but make the selected v1.17 default explicit.
  std::string position_topic_ = "/fmu/out/vehicle_local_position_v1";
  std::string attitude_topic_ = "/fmu/out/vehicle_attitude";
  std::string angular_velocity_topic_ = "/fmu/out/vehicle_angular_velocity";
  std::string output_topic_ = "vehicle_state";
  std::string frame_id_ = "map";
  double state_timeout_seconds_ = 0.25;
  double max_sample_skew_seconds_ = 0.10;
  double publish_rate_hz_ = 50.0;
  rclcpp::Subscription<Px4Position>::SharedPtr position_subscription_;
  rclcpp::Subscription<Px4Attitude>::SharedPtr attitude_subscription_;
  rclcpp::Subscription<Px4AngularVelocity>::SharedPtr angular_velocity_subscription_;
  rclcpp::Publisher<State>::SharedPtr state_publisher_;
  rclcpp::Publisher<Diagnostics>::SharedPtr diagnostics_publisher_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_diagnostics_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VehicleStateBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
