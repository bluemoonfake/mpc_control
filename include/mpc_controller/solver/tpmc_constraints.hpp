#pragma once

#include "mpc_controller/solver/tpmc_types.hpp"

#include <string>

namespace mpc_controller::tpmc {

double stateConstraintViolation(const State &state,
                                const Configuration &configuration) noexcept;

double inputConstraintViolation(const Input &input,
                                const Configuration &configuration) noexcept;

double
inputTransitionConstraintViolation(const Input &previous_input,
                                   const Input &input,
                                   const Configuration &configuration) noexcept;

double constraintViolation(const State &state, const Input &input,
                           const Configuration &configuration) noexcept;

bool hasValidCollectiveSpecificForce(
    const State &state, const Configuration &configuration) noexcept;

std::string describeStateViolation(const State &state,
                                   const Configuration &configuration);

std::string describeMeasuredStateViolation(
    const State &state, const Configuration &configuration);

std::string describeInputViolation(const Input &input,
                                   const Configuration &configuration);

std::string
describeInputTransitionViolation(const Input &previous_input,
                                 const Input &input,
                                 const Configuration &configuration);

} // namespace mpc_controller::tpmc
