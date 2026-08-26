#pragma once

#include "mpc_controller/solver/tpmc_types.hpp"

namespace mpc_controller::tpmc {

Vector3 bodyZDirection(const State &state) noexcept;
double tiltAngle(const State &state) noexcept;

State continuousDynamics(const State &state, const Input &input,
                         const ModelParameters &parameters) noexcept;

State integrateErk4(const State &state, const Input &input, double step_seconds,
                    const ModelParameters &parameters) noexcept;

} // namespace mpc_controller::tpmc
