#pragma once

#include <mrs_mpc_solvers/mpc_controller.h>

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <memory>

namespace mpc_control
{

inline constexpr std::size_t kSolverHorizon = 26;
inline constexpr std::size_t kStateDimension = 3;

using SolverState = Eigen::Vector3d;
using SolverReferenceHorizon = std::array<SolverState, kSolverHorizon>;
using PredictionHorizon = std::array<SolverState, kSolverHorizon>;

struct SolverConfiguration
{
  int max_iterations = 45;
  std::array<double, kStateDimension> stage_weights{500.0, 100.0, 100.0};
  std::array<double, kStateDimension> terminal_weights{1000.0, 300.0, 300.0};

  double dt_first = 0.01;
  double dt_later = 0.20;
  double model_p1 = 0.0;
  double model_p2 = 1.0;

  double max_speed = 2.0;
  double max_acceleration = 999.0;
  double max_control = 2.0;
  double max_control_rate = 5.0;
};

struct SolverRequest
{
  SolverState initial_state = SolverState::Zero();
  SolverReferenceHorizon reference{};
  double last_control = 0.0;

  SolverRequest()
  {
    reference.fill(SolverState::Zero());
  }
};

enum class SolverFailureReason
{
  None,
  InvalidConfiguration,
  InitializationFailure,
  NonFiniteInput,
  SolverDidNotConverge,
  NonFiniteOutput,
  OutputOutOfBounds,
  BackendException,
};

struct SolverResult
{
  bool valid = false;
  SolverFailureReason failure_reason = SolverFailureReason::InvalidConfiguration;
  int iterations = 0;
  double first_control = 0.0;
  PredictionHorizon prediction{};

  SolverResult()
  {
    prediction.fill(SolverState::Zero());
  }
};

class SolverBackend
{
public:
  virtual ~SolverBackend() = default;

  // This is the control-boundary API. It must never propagate an exception.
  virtual SolverResult solve(const SolverRequest& request) noexcept = 0;
};

class MrsSolverBackend final : public SolverBackend
{
public:
  explicit MrsSolverBackend(const SolverConfiguration& configuration);
  ~MrsSolverBackend() override;

  MrsSolverBackend(const MrsSolverBackend&) = delete;
  MrsSolverBackend& operator=(const MrsSolverBackend&) = delete;

  bool isConfigured() const noexcept;
  const SolverConfiguration& configuration() const noexcept;

  SolverResult solve(const SolverRequest& request) noexcept override;

private:
  struct Workspace;

  static bool isConfigurationValid(const SolverConfiguration& configuration) noexcept;
  static bool isRequestFinite(const SolverRequest& request) noexcept;
  bool predictionWithinLimits(const PredictionHorizon& prediction) const noexcept;
  bool firstControlWithinLimits(double control, double last_control) const noexcept;

  SolverConfiguration configuration_;
  bool configured_ = false;
  bool initialization_failed_ = false;

  std::unique_ptr<mrs_mpc_solvers::mpc_controller::Solver> solver_;
  std::unique_ptr<Workspace> workspace_;
};

}  // namespace mpc_control
