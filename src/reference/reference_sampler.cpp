#include <mpc_control/reference_sampler.hpp>

#include <algorithm>
#include <cmath>

namespace mpc_control
{

namespace
{

constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr double kTwoPi = 2.0 * kPi;

}  // namespace

bool ReferenceSampler::validGrid(
    const ReferenceSamplerConfig& configuration) noexcept
{
  return std::isfinite(configuration.dt_first)
      && std::isfinite(configuration.dt_later)
      && configuration.dt_first > 0.0  //dt1
      && configuration.dt_later > 0.0; //dt2
}

double ReferenceSampler::wrapYaw(const double yaw) noexcept
{
  double wrapped = std::fmod(yaw + kPi, kTwoPi);
  if (wrapped < 0.0) {
    wrapped += kTwoPi;
  }
  return wrapped - kPi;
}

ReferencePoint ReferenceSampler::interpolate(
    const ReferencePoint& lower,  //thoi diem truoc
    const ReferencePoint& upper,  //thoi diem sau
    const double time_seconds) noexcept
{
  const double dt = upper.time_seconds - lower.time_seconds;
  const double alpha = (time_seconds - lower.time_seconds) / dt;
  const double yaw_delta = wrapYaw(upper.yaw - lower.yaw);

  ReferencePoint result;
  result.time_seconds = time_seconds;
  result.position = lower.position + alpha * (upper.position - lower.position);
  result.velocity = lower.velocity + alpha * (upper.velocity - lower.velocity);
  result.acceleration =
      lower.acceleration + alpha * (upper.acceleration - lower.acceleration);
  result.yaw = wrapYaw(lower.yaw + alpha * yaw_delta);
  result.yaw_rate = lower.yaw_rate + alpha * (upper.yaw_rate - lower.yaw_rate);
  return result;
}

ReferenceSampleResult ReferenceSampler::sample(
    const ReferenceTrajectory& trajectory,
    const double current_time_seconds,
    const ReferenceSamplerConfig& configuration) const noexcept
{
  ReferenceSampleResult result;

  if (!validGrid(configuration) || !std::isfinite(current_time_seconds)) {
    result.error = ReferenceError::InvalidGrid;
    return result;
  }
//check input valid or not
  const ReferenceValidationResult validation =
      trajectory.validate(configuration.validation);
  if (!validation.valid) {
    result.error = validation.error;
    result.error_index = validation.index;
    return result;
  }
//get points from trajectory
  const auto& points = trajectory.points();
  result.horizon.current_time_seconds = current_time_seconds;
  result.horizon.relative_times[0] = configuration.dt_first;
  for (std::size_t index = 1; index < kReferenceHorizonLength; ++index) {
    result.horizon.relative_times[index] =
        configuration.dt_first
        + static_cast<double>(index) * configuration.dt_later;
  }

  for (std::size_t index = 0; index < kReferenceHorizonLength; ++index) {
    const double query_time =
        current_time_seconds + result.horizon.relative_times[index];

    if (query_time < points.front().time_seconds) {
      result.error = ReferenceError::BeforeStart;
      result.error_index = index;
      return result;
    }

    if (query_time > points.back().time_seconds) {
      if (!configuration.hold_after_end) {
        result.error = ReferenceError::AfterEnd;
        result.error_index = index;
        return result;
      }
      result.horizon.samples[index] = points.back();
      result.horizon.samples[index].time_seconds = query_time;
      continue;
    }

    const auto upper = std::upper_bound(
        points.begin(),
        points.end(),
        query_time,
        [](const double time, const ReferencePoint& point) {
          return time < point.time_seconds;
        });

    if (upper == points.begin()) {
      result.horizon.samples[index] = points.front();
      result.horizon.samples[index].time_seconds = query_time;
    } else if (upper == points.end()) {
      result.horizon.samples[index] = points.back();
      result.horizon.samples[index].time_seconds = query_time;
    } else {
      result.horizon.samples[index] = interpolate(*(upper - 1), *upper, query_time);
    }
  }

  result.valid = true;
  result.error = ReferenceError::None;
  return result;
}

}  // namespace mpc_control
