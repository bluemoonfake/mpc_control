#include "mpc_controller/solver/tpmc_solver.hpp"

namespace mpc_controller::tpmc {

Application::Application(std::unique_ptr<Solver> solver)
    : solver_(std::move(solver)) {}

bool Application::configured() const noexcept {
  return solver_ != nullptr && solver_->configured();
}

void Application::reset() noexcept {
  if (solver_ != nullptr) {
    solver_->reset();
  }
}

SolveResult Application::solve(const SolveRequest &request) noexcept {
  if (solver_ == nullptr) {
    SolveResult result;
    result.status = SolverStatus::not_initialized;
    result.detail = "TMPC solver was not constructed";
    return result;
  }
  if (!finite(request.initial_state) || !finite(request.previous_input)) {
    SolveResult result;
    result.status = SolverStatus::invalid_input;
    result.detail = "Initial state or previous input is non-finite";
    return result;
  }
  for (const auto &reference : request.reference) {
    if (!finite(reference.state) || !finite(reference.input)) {
      SolveResult result;
      result.status = SolverStatus::invalid_input;
      result.detail = "Reference contains a non-finite value";
      return result;
    }
  }
  return solver_->solve(request);
}

const char *Application::backendName() const noexcept {
  if (solver_ == nullptr) {
    return "unknown";
  }
  return solver_->backendName();
}

} // namespace mpc_controller::tpmc
