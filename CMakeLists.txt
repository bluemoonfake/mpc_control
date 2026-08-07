cmake_minimum_required(VERSION 3.16)

project(mpc_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(MPC_CONTROL_BUILD_TESTS "Build mpc_control tests" ON)
option(MPC_CONTROL_ENABLE_SANITIZERS "Enable AddressSanitizer and UBSan" OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Build only the controller part of the pinned MRS dependency. The upstream
# package also builds MpcTrackerSolver unconditionally, which is unnecessary
# for the controller contract tests and roughly doubles generated-code build
# time and memory use.
set(MRS_MPC_SOLVERS_SOURCE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/src/mrs_mpc_solvers"
    CACHE PATH "Path to the pinned mrs_mpc_solvers checkout")

if(NOT EXISTS "${MRS_MPC_SOLVERS_SOURCE_DIR}/include/mrs_mpc_solvers/mpc_controller.h")
  message(FATAL_ERROR
    "mrs_mpc_solvers was not found at ${MRS_MPC_SOLVERS_SOURCE_DIR}. "
    "Initialize the pinned dependency before configuring mpc_control.")
endif()

add_library(mrs_mpc_controller_solver STATIC
  "${MRS_MPC_SOLVERS_SOURCE_DIR}/src/mpc_controller/mpc_controller_solver.cpp"
  "${MRS_MPC_SOLVERS_SOURCE_DIR}/src/mpc_controller/solver/ldl.cpp"
  "${MRS_MPC_SOLVERS_SOURCE_DIR}/src/mpc_controller/solver/matrix_support.cpp"
  "${MRS_MPC_SOLVERS_SOURCE_DIR}/src/mpc_controller/solver/solver.cpp"
  "${MRS_MPC_SOLVERS_SOURCE_DIR}/src/mpc_controller/solver/util.cpp"
)

target_include_directories(mrs_mpc_controller_solver
  PUBLIC "${MRS_MPC_SOLVERS_SOURCE_DIR}/include"
)

target_link_libraries(mrs_mpc_controller_solver PUBLIC Eigen3::Eigen)

add_library(mpc_solver_backend STATIC
  src/solver/mrs_solver_backend.cpp
)

target_include_directories(mpc_solver_backend
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_solver_backend
  PUBLIC
    mrs_mpc_controller_solver
    Eigen3::Eigen
)

add_library(mpc_reference STATIC
  src/reference/reference_trajectory.cpp
  src/reference/reference_sampler.cpp
)

target_include_directories(mpc_reference
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_reference PUBLIC Eigen3::Eigen)

add_library(mpc_model STATIC
  src/model/discrete_virtual_axis_model.cpp
)

target_include_directories(mpc_model
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_model PUBLIC Eigen3::Eigen)

if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mpc_model PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
endif()

add_library(mpc_mil STATIC
  src/mil/mil_runner.cpp
)

target_include_directories(mpc_mil
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_mil
  PUBLIC
    mpc_core
    mpc_model
)

if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mpc_mil PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
endif()

add_library(mpc_core STATIC
  src/core/mpc_trajectory_core.cpp
)

target_include_directories(mpc_core
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_core
  PUBLIC
    mpc_reference
    mpc_solver_backend
)

if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mrs_mpc_controller_solver PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
endif()

# Generated CVXGEN code is third-party code. Apply strict warnings only to
# mpc_control-owned sources; otherwise generated code produces unactionable
# warnings and obscures problems in the adapter/tests.
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mrs_mpc_controller_solver PRIVATE -w)
endif()

add_library(mpc_timing_benchmark_lib STATIC
  src/benchmark/timing_benchmark.cpp
)

target_include_directories(mpc_timing_benchmark_lib
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_timing_benchmark_lib
  PUBLIC mpc_core
)

if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mpc_timing_benchmark_lib PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
endif()

add_library(mpc_monte_carlo_campaign_lib STATIC
  src/benchmark/monte_carlo_campaign.cpp
)

target_include_directories(mpc_monte_carlo_campaign_lib
  PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(mpc_monte_carlo_campaign_lib
  PUBLIC mpc_core
)

if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(mpc_monte_carlo_campaign_lib PRIVATE
    -fsanitize=address,undefined
    -fno-omit-frame-pointer
  )
endif()

if(MPC_CONTROL_BUILD_TESTS)
  include(CTest)
  enable_testing()

  find_package(GTest REQUIRED)
  include(GoogleTest)
  include(cmake/warnings.cmake)

  add_executable(mrs_solver_contract_test
    test/solver_contract/mrs_solver_contract_test.cpp
  )

  target_link_libraries(mrs_solver_contract_test
    PRIVATE
      mrs_mpc_controller_solver
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mrs_solver_contract_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mrs_solver_contract_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mrs_solver_contract_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mrs_solver_contract_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mrs_solver_backend_test
    test/solver_contract/mrs_solver_backend_test.cpp
  )

  target_include_directories(mrs_solver_backend_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test/solver_contract"
  )

  target_link_libraries(mrs_solver_backend_test
    PRIVATE
      mpc_solver_backend
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mrs_solver_backend_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mrs_solver_backend_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mrs_solver_backend_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mrs_solver_backend_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_reference_test
    test/reference/reference_test.cpp
  )

  target_link_libraries(mpc_reference_test
    PRIVATE
      mpc_reference
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_reference_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_reference_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_reference_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_reference_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_model_test
    test/model/discrete_virtual_axis_model_test.cpp
  )

  target_link_libraries(mpc_model_test
    PRIVATE
      mpc_model
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_model_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_model_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_model_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_model_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_mil_test
    test/mil/mil_runner_test.cpp
  )

  target_link_libraries(mpc_mil_test
    PRIVATE
      mpc_mil
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_mil_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_mil_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_mil_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_mil_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_core_test
    test/core/mpc_trajectory_core_test.cpp
  )

  target_include_directories(mpc_core_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test/solver_contract"
  )

  target_link_libraries(mpc_core_test
    PRIVATE
      mpc_core
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_core_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_core_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_core_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_core_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_core_property_test
    test/core/mpc_trajectory_core_property_test.cpp
  )

  target_include_directories(mpc_core_property_test
    PRIVATE "${CMAKE_CURRENT_SOURCE_DIR}/test/solver_contract"
  )

  target_link_libraries(mpc_core_property_test
    PRIVATE
      mpc_core
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_core_property_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_core_property_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_core_property_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_core_property_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_timing_benchmark_test
    test/benchmark/timing_benchmark_test.cpp
  )

  target_link_libraries(mpc_timing_benchmark_test
    PRIVATE
      mpc_timing_benchmark_lib
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_timing_benchmark_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_timing_benchmark_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_timing_benchmark_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_timing_benchmark_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_monte_carlo_test
    test/monte_carlo/mil_monte_carlo_test.cpp
  )

  target_link_libraries(mpc_monte_carlo_test
    PRIVATE
      mpc_core
      GTest::gtest_main
  )

  mpc_control_enable_warnings(mpc_monte_carlo_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_monte_carlo_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_monte_carlo_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(mpc_monte_carlo_test
    DISCOVERY_TIMEOUT 300
  )

  add_executable(monte_carlo_campaign_report_test
    test/benchmark/monte_carlo_campaign_report_test.cpp
  )

  target_link_libraries(monte_carlo_campaign_report_test
    PRIVATE
      mpc_monte_carlo_campaign_lib
      GTest::gtest_main
  )

  mpc_control_enable_warnings(monte_carlo_campaign_report_test)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(monte_carlo_campaign_report_test PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(monte_carlo_campaign_report_test PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  gtest_discover_tests(monte_carlo_campaign_report_test
    DISCOVERY_TIMEOUT 30
  )

  add_executable(mpc_monte_carlo_campaign
    tools/monte_carlo_campaign_main.cpp
  )

  target_link_libraries(mpc_monte_carlo_campaign
    PRIVATE mpc_monte_carlo_campaign_lib
  )

  mpc_control_enable_warnings(mpc_monte_carlo_campaign)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_monte_carlo_campaign PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_monte_carlo_campaign PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  add_executable(mpc_full_horizon_load_campaign
    tools/full_horizon_load_campaign_main.cpp
  )

  target_link_libraries(mpc_full_horizon_load_campaign
    PRIVATE mpc_monte_carlo_campaign_lib
  )

  mpc_control_enable_warnings(mpc_full_horizon_load_campaign)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_full_horizon_load_campaign PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_full_horizon_load_campaign PRIVATE
      -fsanitize=address,undefined
    )
  endif()

  add_executable(mpc_timing_benchmark
    tools/timing_benchmark_main.cpp
  )

  target_link_libraries(mpc_timing_benchmark
    PRIVATE mpc_timing_benchmark_lib
  )

  mpc_control_enable_warnings(mpc_timing_benchmark)

  if(MPC_CONTROL_ENABLE_SANITIZERS AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(mpc_timing_benchmark PRIVATE
      -fsanitize=address,undefined
      -fno-omit-frame-pointer
    )
    target_link_options(mpc_timing_benchmark PRIVATE
      -fsanitize=address,undefined
    )
  endif()
endif()
