#pragma once

#include "mpc_controller/solver/tpmc_solver.hpp"

#include <memory>
#include <string>

namespace mpc_controller::tpmc {

class AcadosTpmcSolver final : public Solver {
public:
  explicit AcadosTpmcSolver(const Configuration &configuration);
  ~AcadosTpmcSolver() override;

  bool configured() const noexcept override;
  void reset() noexcept override;
  SolveResult solve(const SolveRequest &request) noexcept override;

  const char *backendName() const noexcept;
  const Configuration &configuration() const noexcept;
  const std::string &dependencyStatus() const noexcept;

private:
  struct Runtime;

  Configuration configuration_{};
  std::unique_ptr<Runtime> runtime_;
  StatePrediction warm_states_{};
  InputPrediction warm_inputs_{};
  bool has_warm_start_ = false;
  bool configured_ = false;
  std::string dependency_status_;
};

} // namespace mpc_controller::tpmc
