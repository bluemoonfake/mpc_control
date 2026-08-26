#pragma once

#include "mpc_controller/solver/tpmc_types.hpp"

#include <memory>

namespace mpc_controller::tpmc {

class Solver {
public:
  virtual ~Solver() = default;

  Solver(const Solver &) = delete;
  Solver &operator=(const Solver &) = delete;

  virtual bool configured() const noexcept = 0;
  virtual void reset() noexcept = 0;
  virtual SolveResult solve(const SolveRequest &request) noexcept = 0;
  virtual const char *backendName() const noexcept = 0;

protected:
  Solver() = default;
};

class Application final {
public:
  explicit Application(std::unique_ptr<Solver> solver);

  Application(const Application &) = delete;
  Application &operator=(const Application &) = delete;

  bool configured() const noexcept;
  void reset() noexcept;
  SolveResult solve(const SolveRequest &request) noexcept;
  const char *backendName() const noexcept;

private:
  std::unique_ptr<Solver> solver_;
};

} // namespace mpc_controller::tpmc
