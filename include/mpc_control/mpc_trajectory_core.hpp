#pragma once

#include "reference_sampler.hpp"
#include "solver_backend.hpp"

#include <Eigen/Core>

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>

namespace mpc_control
{

struct VehicleState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
};

struct VirtualTrajectoryState
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  double trajectory_time_seconds = 0.0;
};

enum class MpcFailureReason
{
  None,
  InvalidConfiguration,
  NotConfigured,
  AlreadyActive,
  NotActive,
  InvalidVehicleState,
  InvalidReferenceHorizon,
  TimeMismatch,
  SolverFailureX,
  SolverFailureY,
  SolverFailureZ,
  NonFiniteOutput,
  BackendException,
};

struct MpcCoreConfiguration
{
  SolverConfiguration horizontal_solver{};
  SolverConfiguration vertical_solver{};
  double time_tolerance_seconds = 1.0e-6;
  bool use_measured_acceleration_on_activate = false;

  MpcCoreConfiguration()
  {
    vertical_solver.model_p1 = 0.5;
    vertical_solver.model_p2 = 0.5;
  }
};

struct TrajectoryCommand
{
  Eigen::Vector3d position = Eigen::Vector3d::Zero();
  Eigen::Vector3d velocity = Eigen::Vector3d::Zero();
  Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
  double yaw = 0.0;
  double yaw_rate = 0.0;
  bool valid = false;
  MpcFailureReason failure_reason = MpcFailureReason::NotConfigured;
};

struct MpcDiagnostics
{
  std::uint64_t update_count = 0;
  bool state_reset = false;
  std::array<int, 3> solver_iterations{0, 0, 0};
  std::array<SolverFailureReason, 3> solver_failure_reasons{
      SolverFailureReason::InvalidConfiguration,
      SolverFailureReason::InvalidConfiguration,
      SolverFailureReason::InvalidConfiguration};
};

struct MpcUpdateResult
{
  bool valid = false;
  MpcFailureReason failure_reason = MpcFailureReason::NotConfigured;
  TrajectoryCommand command{};
  std::array<TrajectoryCommand, kReferenceHorizonLength> prediction{};
  MpcDiagnostics diagnostics{};
};

struct MpcConfigureResult
{
  bool valid = false;
  MpcFailureReason failure_reason = MpcFailureReason::InvalidConfiguration;
};

struct MpcActivationResult
{
  bool valid = false;
  MpcFailureReason failure_reason = MpcFailureReason::NotConfigured;
};

class MpcTrajectoryCore
{
public:
  MpcTrajectoryCore();
  explicit MpcTrajectoryCore(const MpcCoreConfiguration& configuration);
  MpcTrajectoryCore(
      const MpcCoreConfiguration& configuration,
      const std::array<SolverBackend*, 3>& external_backends);
  ~MpcTrajectoryCore() = default;

  MpcTrajectoryCore(const MpcTrajectoryCore&) = delete;
  MpcTrajectoryCore& operator=(const MpcTrajectoryCore&) = delete;

  MpcConfigureResult configure(
      const MpcCoreConfiguration& configuration) noexcept;

  MpcActivationResult activate(
      const VehicleState& measured_state,
      double reference_time_seconds) noexcept;

  MpcUpdateResult update(
      const ReferenceHorizon& reference_horizon,
      double update_time_seconds) noexcept;

  void reset(
      const VehicleState& measured_state,
      double reference_time_seconds) noexcept;

  bool configured() const noexcept;
  bool active() const noexcept;
  const VirtualTrajectoryState& virtualState() const noexcept;

private:
  static bool validConfiguration(
      const MpcCoreConfiguration& configuration) noexcept;
  static bool finiteVehicleState(
      const VehicleState& state,
      bool require_acceleration) noexcept;
  static bool validReferenceHorizon(
      const ReferenceHorizon& horizon) noexcept;
  static Eigen::Vector3d axisState(
      const Eigen::Vector3d& vector,
      std::size_t axis) noexcept;
  static MpcFailureReason axisFailure(std::size_t axis) noexcept;

  void clearRuntimeState() noexcept;
  bool initializeVirtualState(
      const VehicleState& measured_state,
      double reference_time_seconds) noexcept;

  MpcCoreConfiguration configuration_{};
  bool configured_ = false;
  bool active_ = false;
  bool external_backends_ = false;
  bool state_reset_pending_ = false;

  std::array<std::unique_ptr<SolverBackend>, 3> owned_backends_{};
  std::array<SolverBackend*, 3> backends_{nullptr, nullptr, nullptr};
  std::array<double, 3> last_controls_{0.0, 0.0, 0.0};

  VirtualTrajectoryState virtual_state_{};
  std::uint64_t update_count_ = 0;
};

}  // namespace mpc_control
