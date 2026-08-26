#pragma once

#include "mpc_controller/solver/tpmc_types.hpp"

namespace mpc_controller::tpmc {

bool finite(const ReferencePoint &point) noexcept;
bool validTrajectory(const ReferenceTrajectory &trajectory) noexcept;
double shortestAngle(double from, double to) noexcept;
double rebaseAngle(double continuous_reference, double wrapped_angle) noexcept;

bool sampleReference(const ReferenceTrajectory &trajectory,
                     double time_from_start, ReferencePoint &output) noexcept;

bool buildReferenceHorizon(const ReferenceTrajectory &trajectory,
                           double trajectory_age_seconds,
                           double sample_time_seconds, double maximum_tilt_rad,
                           double gravity_m_s2,
                           ReferenceHorizon &output) noexcept;

} // namespace mpc_controller::tpmc
