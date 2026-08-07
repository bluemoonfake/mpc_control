#include <cmath>
#include <cstdio>

constexpr int kHorizon = 8;

extern "C" int reference_circle(
    double time_seconds, double position[3], double velocity[3],
    double acceleration[3], double* yaw) noexcept;

extern "C" int mpc_update(
    const double measured_position[3], const double measured_velocity[3],
    const double reference_position[3][kHorizon],
    const double reference_velocity[3][kHorizon],
    const double reference_acceleration[3][kHorizon],
    double acceleration_command[3], double* last_yaw_command) noexcept;

extern "C" int acceleration_to_attitude(
    const double acceleration_enu[3], double yaw_enu,
    double quaternion_body_to_ned[4], double thrust_body_frd[3]) noexcept;

int main()
{
  double measured_position[3] = {2.0, 0.0, 1.0};
  double measured_velocity[3] = {0.0, 0.0, 0.0};
  double yaw = 0.0;

  for (int iteration = 0; iteration < 250; ++iteration) {
    const double time = 0.02 * static_cast<double>(iteration);
    double reference_position[3][kHorizon]{};
    double reference_velocity[3][kHorizon]{};
    double reference_acceleration[3][kHorizon]{};

    for (int step = 0; step < kHorizon; ++step) {
      double position[3]{};
      double velocity[3]{};
      double acceleration[3]{};
      double reference_yaw = 0.0;
      reference_circle(time + 0.02 * static_cast<double>(step), position,
          velocity, acceleration, &reference_yaw);
      yaw = reference_yaw;
      for (int axis = 0; axis < 3; ++axis) {
        reference_position[axis][step] = position[axis];
        reference_velocity[axis][step] = velocity[axis];
        reference_acceleration[axis][step] = acceleration[axis];
      }
    }

    double acceleration_command[3]{};
    if (!mpc_update(measured_position, measured_velocity, reference_position,
        reference_velocity, reference_acceleration, acceleration_command,
        &yaw)) {
      return 1;
    }

    double quaternion[4]{};
    double thrust[3]{};
    if (!acceleration_to_attitude(
            acceleration_command, yaw, quaternion, thrust)) {
      return 1;
    }

    measured_position[0] += 0.02 * measured_velocity[0]
        + 0.5 * 0.02 * 0.02 * acceleration_command[0];
    measured_position[1] += 0.02 * measured_velocity[1]
        + 0.5 * 0.02 * 0.02 * acceleration_command[1];
    measured_position[2] += 0.02 * measured_velocity[2]
        + 0.5 * 0.02 * 0.02 * acceleration_command[2];
    for (int axis = 0; axis < 3; ++axis) {
      measured_velocity[axis] += 0.02 * acceleration_command[axis];
    }

    if (iteration % 50 == 0) {
      std::printf("t=%.2f a=[%.3f %.3f %.3f] q=[%.3f %.3f %.3f %.3f]\n",
          time, acceleration_command[0], acceleration_command[1],
          acceleration_command[2], quaternion[0], quaternion[1],
          quaternion[2], quaternion[3]);
    }
  }
  return 0;
}
