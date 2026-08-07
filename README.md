# mpc_control

MPC controller development workspace. The current gate characterizes the
pinned `mrs_mpc_solvers` controller backend without ROS, PX4, or KR runtime
integration.

## Prerequisites

- CMake 3.16 or newer
- C++20 compiler
- Eigen3
- GoogleTest
- the pinned `src/mrs_mpc_solvers` checkout
- the pinned PX4 official `v1.17.0` submodule at `third_party/PX4-Autopilot`
- ROS 2 Jazzy under `/opt/ros/jazzy`
- `colcon` for building the pinned `third_party/px4_msgs` overlay

Initialize the root repository submodules after cloning:

```bash
git submodule update --init src/mrs_mpc_solvers third_party/PX4-Autopilot
```

The PX4 submodule is pinned by the superproject to the official `v1.17.0`
commit. The separate `manifests/dependencies.repos` file remains available for
clean external workspaces and CI.

For the PX4 `gz_x500` SITL build, initialize PX4's recorded source
dependencies, excluding the legacy FlightGear and Gazebo Classic simulators:

```bash
px4_dir=third_party/PX4-Autopilot
mapfile -t px4_paths < <(
  git -C "$px4_dir" config --file .gitmodules --get-regexp '\.path$' \
    | awk '$2 != "Tools/simulation/flightgear/flightgear_bridge" && \
           $2 != "Tools/simulation/gazebo-classic/sitl_gazebo-classic" {print $2}'
)
git -C "$px4_dir" submodule sync -- "${px4_paths[@]}"
git -C "$px4_dir" submodule update --init --force --checkout "${px4_paths[@]}"
```

Build from the project root so the Gazebo runtime wrapper is sourced with the
correct path:

```bash
source tools/px4_gazebo_harmonic_env.sh
(cd third_party/PX4-Autopilot && make px4_sitl gz_x500)
```

The pinned M0 container baseline is under `containers/m0/`. Build it with:

```bash
docker build --pull=false \
  --file containers/m0/Dockerfile \
  --tag mpc-control:m0-20260805 .
```

Build and source the official PX4 v1.17.0 message overlay on the host:

```bash
source /opt/ros/jazzy/setup.bash
colcon --log-base log/ros_px4_msgs build \
  --base-paths third_party/px4_msgs \
  --build-base build/ros_px4_msgs \
  --install-base install/ros_px4_msgs \
  --merge-install
source install/ros_px4_msgs/setup.bash
```

## Build and run the solver contract tests

```bash
cmake -S . -B build/contract -DCMAKE_BUILD_TYPE=Debug
cmake --build build/contract --parallel 2
ctest --test-dir build/contract --output-on-failure
```

Run each discovered test 100 times:

```bash
ctest --test-dir build/contract --repeat until-fail:100 --output-on-failure
```

Run with AddressSanitizer and UndefinedBehaviorSanitizer:

```bash
cmake -S . -B build/sanitize \
  -DCMAKE_BUILD_TYPE=Debug \
  -DMPC_CONTROL_ENABLE_SANITIZERS=ON
cmake --build build/sanitize --parallel 2
ctest --test-dir build/sanitize --output-on-failure
```

## Coverage

For the current contract gate, capture only the solver target. Capturing the
test target itself can trigger an lcov/GCC 13 line-mapping mismatch and is not
useful as production-code coverage.

```bash
sudo apt install lcov

cmake -S . -B build/coverage \
  -DCMAKE_BUILD_TYPE=Debug \
  -DCMAKE_CXX_FLAGS="--coverage -O0 -g" \
  -DCMAKE_EXE_LINKER_FLAGS="--coverage"
cmake --build build/coverage --parallel 2
ctest --test-dir build/coverage --output-on-failure

lcov --capture \
  --directory build/coverage/CMakeFiles/mrs_mpc_controller_solver.dir \
  --output-file build/coverage/coverage.solver.info \
  --branch-coverage

lcov --remove build/coverage/coverage.solver.info \
  '/usr/*' \
  --output-file build/coverage/coverage.solver.filtered.info \
  --branch-coverage

genhtml build/coverage/coverage.solver.filtered.info \
  --output-directory build/coverage/html \
  --branch-coverage --legend
```

Open `build/coverage/html/index.html` in a browser. The current solver
characterization run produced 76.9% line, 86.0% function, and 52.2% branch
coverage. These numbers describe the generated MRS controller solver; future
coverage gates should focus on the new `mpc_control_core` and backend code.

Only `MpcControllerSolver` is compiled by this root project. The unrelated
MRS tracker solver is deliberately excluded from this test gate.
