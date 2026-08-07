#include <cmath>

namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kRadius = 2.0;
constexpr double kPeriod = 60.0;
constexpr double kOmega = 2.0 * kPi / kPeriod;
constexpr double kCenter[3] = {0.0, 0.0, 1.0};

}  // namespace

extern "C" int reference_circle(
    const double time_seconds,
    double position[3],
    double velocity[3],
    double acceleration[3],
    double* yaw) noexcept
{
  if (position == nullptr || velocity == nullptr || acceleration == nullptr
      || yaw == nullptr || !std::isfinite(time_seconds)) {
    return 0;
  }

  const double phase = kOmega * time_seconds;
  const double c = std::cos(phase);
  const double s = std::sin(phase);

  position[0] = kCenter[0] + kRadius * c;
  position[1] = kCenter[1] + kRadius * s;
  position[2] = kCenter[2];

  velocity[0] = -kRadius * kOmega * s;
  velocity[1] = kRadius * kOmega * c;
  velocity[2] = 0.0;

  acceleration[0] = -kRadius * kOmega * kOmega * c;
  acceleration[1] = -kRadius * kOmega * kOmega * s;
  acceleration[2] = 0.0;

  *yaw = std::atan2(velocity[1], velocity[0]);
  return 1;
}

