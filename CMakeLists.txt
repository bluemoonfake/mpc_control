cmake_minimum_required(VERSION 3.16)

project(mpc_controller)

# The MPC hot path is Eigen-heavy and must run inside a 20 ms control period.
# Keep Release as the default while allowing an explicit build type to override it.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

option(MPC_ENABLE_HPIPM
  "Build HPIPM with BLASFEO and use the OCP-QP backend for runtime MPC"
  ON)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  set(_mpc_default_hpipm_target "GENERIC")
  set(_mpc_default_blasfeo_target "ARMV8A_ARM_CORTEX_A76")
else()
  set(_mpc_default_hpipm_target "AVX")
  set(_mpc_default_blasfeo_target "X64_INTEL_HASWELL")
endif()
set(MPC_HPIPM_TARGET "${_mpc_default_hpipm_target}" CACHE STRING
  "HPIPM core target: AVX or GENERIC")
set(MPC_BLASFEO_TARGET "${_mpc_default_blasfeo_target}" CACHE STRING
  "BLASFEO target architecture")
option(MPC_ENABLE_TINYMPC
  "Build the pinned TinyMPC capability probe; keep runtime integration disabled"
  OFF)
option(MPC_BUILD_SOLVER_BENCHMARK
  "Build the deterministic offline solver benchmark"
  OFF)

if(MPC_ENABLE_HPIPM)
  # HPIPM is built as an OCP-QP backend.  Keep both revisions pinned so an
  # offline/embedded build cannot silently change numerical behavior.
  set(TARGET "${MPC_BLASFEO_TARGET}" CACHE STRING "" FORCE)
  set(BLASFEO_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BLASFEO_TESTING OFF CACHE BOOL "" FORCE)
  set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(blasfeo
    URL https://github.com/giaf/blasfeo/archive/60493284741b3e4e5b0bf25155221d4b5f0b2232.tar.gz
    URL_HASH SHA256=e6d28ff323693f54dc47f10a0bc86bf9a3441ed7881c5caa8500954063b8d81a
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(blasfeo)

  set(TARGET "${MPC_HPIPM_TARGET}" CACHE STRING "" FORCE)
  set(HPIPM_TESTING OFF CACHE BOOL "" FORCE)
  set(HPIPM_FIND_BLASFEO OFF CACHE BOOL "" FORCE)
  set(BLASFEO_PATH "${blasfeo_SOURCE_DIR}" CACHE PATH "" FORCE)
  FetchContent_Declare(hpipm
    URL https://github.com/giaf/hpipm/archive/bad1123a96136fce3cdd8bbcbe24b853d012b840.tar.gz
    URL_HASH SHA256=d65ae2bab6467dbfc96759833c351ea28ad3f6d74dd668b185ee144001ad8d05
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(hpipm)
else()
  message(FATAL_ERROR
    "MPC_ENABLE_HPIPM=OFF is unsupported on hpipm_mpc_controller; "
    "this branch requires the HPIPM OCP-QP backend")
endif()

if(MPC_ENABLE_TINYMPC)
  # Keep TinyMPC isolated and opt-in until its stage model reaches numerical
  # Keep TinyMPC isolated and pinned so an optional audit cannot silently
  # change with an upstream branch.
  FetchContent_Declare(tinympc
    URL https://github.com/TinyMPC/TinyMPC/archive/023f36bd5b27267e1a96ed47a774d120b5707f13.tar.gz
    URL_HASH SHA256=1ef63baecb4928d17a383f7a55ead80270bbc880cd9a74eb27bca3d4a8541946
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_GetProperties(tinympc)
  if(NOT tinympc_POPULATED)
    FetchContent_Populate(tinympc)
  endif()
  add_subdirectory(
    "${tinympc_SOURCE_DIR}/src/tinympc"
    "${tinympc_BINARY_DIR}/tinympc"
    EXCLUDE_FROM_ALL)
  add_library(mpc_tinympc INTERFACE)
  target_link_libraries(mpc_tinympc INTERFACE tinympcstatic)
  target_include_directories(mpc_tinympc SYSTEM INTERFACE
    "${tinympc_SOURCE_DIR}/src"
    "${tinympc_SOURCE_DIR}/include/Eigen")
  add_executable(tinympc_capability_probe
    tools/tinympc_capability_probe.cpp)
  target_compile_features(tinympc_capability_probe PRIVATE cxx_std_17)
  target_link_libraries(tinympc_capability_probe PRIVATE mpc_tinympc)

  add_executable(tinympc_parity_gap_test
    tools/tinympc_parity_gap_test.cpp)
  target_compile_features(tinympc_parity_gap_test PRIVATE cxx_std_17)
  target_compile_definitions(tinympc_parity_gap_test PRIVATE
    TINYMPC_API_HEADER_PATH="${tinympc_SOURCE_DIR}/src/tinympc/tiny_api.hpp")
  target_link_libraries(tinympc_parity_gap_test PRIVATE mpc_tinympc)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(px4_ros2_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
  "msg/TrajectoryPoint.msg"
  "msg/ReferenceTrajectory.msg"
  "msg/VehicleState.msg"
  "msg/MpcTranslationalOutput.msg"
  "msg/ForceAttitudeSetpoint.msg"
)

set(srv_files
  "srv/LoadAndStartMission.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(mpc_common INTERFACE)
target_compile_features(mpc_common INTERFACE cxx_std_17)
target_include_directories(mpc_common INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_common INTERFACE Eigen3::Eigen)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_library(mpc_mission STATIC
  src/domain/mission/trajectory.cpp
  src/domain/mission/loader.cpp)
target_compile_features(mpc_mission PUBLIC cxx_std_17)
target_link_libraries(mpc_mission PUBLIC
  mpc_common nlohmann_json::nlohmann_json)

add_library(mpc_solver STATIC
  src/adapters/solver/hpipm.cpp)
target_compile_features(mpc_solver PUBLIC cxx_std_17)
target_link_libraries(mpc_solver PUBLIC mpc_common PRIVATE hpipm)

add_library(mpc_control_application STATIC
  src/application/control_runtime.cpp)
target_compile_features(mpc_control_application PUBLIC cxx_std_17)
target_link_libraries(mpc_control_application PUBLIC mpc_common mpc_solver)

add_library(mpc_mission_application STATIC
  src/application/mission_runtime.cpp)
target_compile_features(mpc_mission_application PUBLIC cxx_std_17)
target_link_libraries(mpc_mission_application PUBLIC mpc_common mpc_mission)

add_executable(mpc_controller_node
  src/nodes/mpc_node.cpp)
target_compile_features(mpc_controller_node PRIVATE cxx_std_17)
ament_target_dependencies(mpc_controller_node rclcpp std_msgs)
target_link_libraries(mpc_controller_node
  mpc_control_application "${cpp_typesupport_target}")

add_executable(reference_generator_node
  src/nodes/reference_node.cpp)
target_compile_features(reference_generator_node PRIVATE cxx_std_17)
ament_target_dependencies(reference_generator_node
  rclcpp std_msgs std_srvs visualization_msgs)
target_link_libraries(reference_generator_node
  mpc_mission_application "${cpp_typesupport_target}")

add_executable(px4_attitude_mode_node
  src/nodes/attitude_node.cpp)
target_compile_features(px4_attitude_mode_node PRIVATE cxx_std_17)
ament_target_dependencies(px4_attitude_mode_node
  rclcpp px4_msgs px4_ros2_cpp std_msgs std_srvs)
target_link_libraries(px4_attitude_mode_node
  mpc_common px4_ros2_cpp::px4_ros2_cpp "${cpp_typesupport_target}")

add_executable(pid_mode_node src/nodes/pid_node.cpp)
ament_target_dependencies(pid_mode_node rclcpp px4_msgs px4_ros2_cpp std_msgs)
target_link_libraries(pid_mode_node
  mpc_common px4_ros2_cpp::px4_ros2_cpp "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pid_reference_test test/unit/compare/pid_test.cpp)
  target_link_libraries(pid_reference_test mpc_common)

  ament_add_gtest(force_mapper_test
    test/unit/adapters/geometry_mapper/force_test.cpp)
  target_link_libraries(force_mapper_test mpc_common)

  ament_add_gtest(control_runtime_test
    test/unit/application/control_runtime_test.cpp)
  target_link_libraries(control_runtime_test mpc_control_application)

  ament_add_gtest(mission_trajectory_test
    test/unit/domain/mission/trajectory_test.cpp)
  target_link_libraries(mission_trajectory_test mpc_mission)

  ament_add_gtest(trajectory_curve_test
    test/unit/domain/model/trajectory_test.cpp)
  target_link_libraries(trajectory_curve_test mpc_mission)

  ament_add_gtest(mission_loader_test
    test/unit/domain/mission/loader_test.cpp)
  target_compile_definitions(mission_loader_test PRIVATE
    MPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(mission_loader_test mpc_mission)

  ament_add_gtest(mission_json_parser_test
    test/integration/mission/mission_json_parser_test.cpp)
  target_compile_definitions(mission_json_parser_test PRIVATE
    MPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(mission_json_parser_test mpc_mission)

  ament_add_gtest(state_mapper_test
    test/unit/adapters/geometry_mapper/state_test.cpp)
  target_link_libraries(state_mapper_test mpc_common)

  ament_add_gtest(mission_progress_tracker_test
    test/unit/domain/mission/tracker_test.cpp)
  target_link_libraries(mission_progress_tracker_test mpc_common)

  ament_add_gtest(limits_test
    test/unit/domain/control/limits_test.cpp)
  target_link_libraries(limits_test mpc_common)

  ament_add_gtest(reference_sampler_test
    test/unit/domain/model/sampler_test.cpp)
  target_link_libraries(reference_sampler_test mpc_common)

  ament_add_gtest(constraint_layout_test
    test/unit/ports/constraint_layout_test.cpp)
  target_link_libraries(constraint_layout_test mpc_common mpc_solver)

  ament_add_gtest(acceleration_envelope_test
    test/unit/ports/acceleration_envelope_test.cpp)
  target_link_libraries(acceleration_envelope_test mpc_common mpc_solver)
endif()

if(MPC_BUILD_SOLVER_BENCHMARK)
  add_executable(mpc_solver_benchmark tools/mpc_solver_benchmark.cpp)
  target_compile_features(mpc_solver_benchmark PRIVATE cxx_std_17)
  target_link_libraries(mpc_solver_benchmark PRIVATE mpc_common mpc_solver)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  mpc_mission
  mpc_solver
  mpc_control_application
  mpc_mission_application
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  reference_generator_node
  mpc_controller_node
  px4_attitude_mode_node
  pid_mode_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  launch/mpc_external_mode.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
