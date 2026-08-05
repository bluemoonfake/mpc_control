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
endif()
