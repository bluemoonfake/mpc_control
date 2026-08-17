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
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(visualization_msgs REQUIRED)

set(msg_files
  "msg/TrajectoryPoint.msg"
  "msg/ReferenceTrajectory.msg"
  "msg/ReferenceStep.msg"
  "msg/VehicleState.msg"
  "msg/MpcTranslationalOutput.msg"
  "msg/ForceAttitudeSetpoint.msg"
  "msg/TorqueThrustSetpointPreview.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

add_library(mpc_core STATIC src/mpc_solver.cpp)
target_compile_features(mpc_core PUBLIC cxx_std_17)
target_include_directories(mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/px4-ros2-interface-lib/px4_ros2_cpp/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_core PUBLIC Eigen3::Eigen PRIVATE osqpstatic)

function(add_mpc_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/px4-ros2-interface-lib/px4_ros2_cpp/include>)
  ament_target_dependencies(${target} rclcpp ${ARGN})
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${target} "${cpp_typesupport_target}")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_mpc_node(reference_generator_node src/reference_generator_node.cpp
  px4_msgs std_srvs visualization_msgs)
add_mpc_node(mpc_controller_node src/mpc_controller_node.cpp std_msgs)
target_link_libraries(mpc_controller_node mpc_core)
add_mpc_node(vehicle_state_bridge_node src/vehicle_state_bridge_node.cpp px4_msgs)
add_mpc_node(px4_torque_thrust_setpoint_node src/px4_torque_thrust_setpoint_node.cpp px4_msgs)
target_link_libraries(px4_torque_thrust_setpoint_node Eigen3::Eigen)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS mpc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(FILES
  launch/mpc_offboard.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
