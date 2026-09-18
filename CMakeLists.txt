cmake_minimum_required(VERSION 3.16)

project(mpc_controller)

# The MPC hot path is Eigen-heavy and must run inside a 20 ms control period.
# Keep Release as the default while allowing an explicit build type to override it.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

include(FetchContent)

# OSQP owns the numerical iteration, factorization, warm start and convergence
# checks. Pin the source so every ROS workspace builds the same solver.
set(OSQP_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(OSQP_BUILD_DEMO_EXE OFF CACHE BOOL "" FORCE)
set(OSQP_BUILD_UNITTESTS OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_PRINTING OFF CACHE BOOL "" FORCE)
set(OSQP_ENABLE_INTERRUPT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(osqp
  URL https://github.com/osqp/osqp/archive/refs/tags/v1.0.0.tar.gz
  URL_HASH SHA256=dd6a1c2e7e921485697d5e7cdeeb043c712526c395b3700601f51d472a7d8e48
  DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
FetchContent_MakeAvailable(osqp)

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
  "msg/MpcMissionWaypoint.msg"
  "msg/MpcMissionPlan.msg"
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
  src/mission/mission_json_parser.cpp
  src/mission/mission_trajectory.cpp
  src/mission/runtime_mission_loader.cpp)
target_compile_features(mpc_mission PUBLIC cxx_std_17)
target_link_libraries(mpc_mission PUBLIC
  mpc_common nlohmann_json::nlohmann_json)

add_library(mpc_solver STATIC
  src/solver/osqp_solver.cpp)
target_compile_features(mpc_solver PUBLIC cxx_std_17)
target_link_libraries(mpc_solver PUBLIC mpc_common PRIVATE osqpstatic)

add_executable(mpc_controller_node
  src/controller/mpc_controller_node.cpp)
target_compile_features(mpc_controller_node PRIVATE cxx_std_17)
ament_target_dependencies(mpc_controller_node rclcpp std_msgs)
target_link_libraries(mpc_controller_node
  mpc_common mpc_solver "${cpp_typesupport_target}")

add_executable(reference_generator_node
  src/mission/reference_generator_node.cpp)
target_compile_features(reference_generator_node PRIVATE cxx_std_17)
ament_target_dependencies(reference_generator_node
  rclcpp std_msgs std_srvs visualization_msgs)
target_link_libraries(reference_generator_node
  mpc_mission "${cpp_typesupport_target}")

add_executable(runtime_mission_client
  src/mission/runtime_mission_client.cpp)
target_compile_features(runtime_mission_client PRIVATE cxx_std_17)
ament_target_dependencies(runtime_mission_client rclcpp)
target_link_libraries(runtime_mission_client "${cpp_typesupport_target}")

add_executable(vehicle_state_bridge_node
  src/px4/vehicle_state_bridge_node.cpp)
target_compile_features(vehicle_state_bridge_node PRIVATE cxx_std_17)
ament_target_dependencies(vehicle_state_bridge_node rclcpp px4_msgs)
target_link_libraries(vehicle_state_bridge_node
  mpc_common "${cpp_typesupport_target}")

add_executable(px4_attitude_mode_node
  src/px4/px4_attitude_mode_node.cpp)
target_compile_features(px4_attitude_mode_node PRIVATE cxx_std_17)
ament_target_dependencies(px4_attitude_mode_node
  rclcpp px4_msgs px4_ros2_cpp std_msgs std_srvs)
target_link_libraries(px4_attitude_mode_node
  mpc_common px4_ros2_cpp::px4_ros2_cpp "${cpp_typesupport_target}")

add_executable(pid_mode_node src/PID_validation/pid_mode_node.cpp)
ament_target_dependencies(pid_mode_node rclcpp px4_msgs px4_ros2_cpp std_msgs)
target_link_libraries(pid_mode_node
  mpc_common px4_ros2_cpp::px4_ros2_cpp "${cpp_typesupport_target}")

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pid_reference_test test/unit/pid_valid/pid_reference_test.cpp)
  target_link_libraries(pid_reference_test mpc_common)

  ament_add_gtest(force_attitude_mapper_test
    test/unit/px4/force_attitude_mapper_test.cpp)
  target_link_libraries(force_attitude_mapper_test mpc_common)

  ament_add_gtest(mission_reference_generator_test
    test/unit/mission/mission_trajectory_test.cpp)
  target_link_libraries(mission_reference_generator_test mpc_mission)

  ament_add_gtest(runtime_mission_loader_test
    test/unit/mission/runtime_mission_loader_test.cpp)
  target_compile_definitions(runtime_mission_loader_test PRIVATE
    MPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(runtime_mission_loader_test mpc_mission)

  ament_add_gtest(mission_json_parser_test
    test/integration/mission/mission_json_parser_test.cpp)
  target_compile_definitions(mission_json_parser_test PRIVATE
    MPC_TEST_FIXTURE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/fixtures")
  target_link_libraries(mission_json_parser_test mpc_mission)

  ament_add_gtest(vehicle_state_mapper_test
    test/unit/px4/vehicle_state_mapper_test.cpp)
  target_link_libraries(vehicle_state_mapper_test mpc_common)

  ament_add_gtest(waypoint_mission_test
    test/unit/controller/waypoint_mission_test.cpp)
  target_link_libraries(waypoint_mission_test mpc_common)

  ament_add_gtest(translational_mpc_goal_test
    test/unit/controller/translational_mpc_goal_test.cpp)
  target_link_libraries(translational_mpc_goal_test mpc_common mpc_solver)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS
  mpc_mission
  mpc_solver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(TARGETS
  reference_generator_node
  runtime_mission_client
  mpc_controller_node
  vehicle_state_bridge_node
  px4_attitude_mode_node
  pid_mode_node
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  launch/mpc_external_mode.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
