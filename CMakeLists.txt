cmake_minimum_required(VERSION 3.16)

project(mpc_controller)

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
  "msg/M3ControlOutput.msg"
  "msg/TorqueThrustSetpointPreview.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

function(add_mpc_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
  ament_target_dependencies(${target} rclcpp ${ARGN})
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${target} "${cpp_typesupport_target}")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

add_mpc_node(reference_generator_node src/reference_generator_node.cpp
  px4_msgs std_srvs visualization_msgs)
add_mpc_node(mpc_controller_node src/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node Eigen3::Eigen)
add_mpc_node(vehicle_state_bridge_node src/vehicle_state_bridge_node.cpp px4_msgs)
add_mpc_node(px4_torque_thrust_setpoint_node src/px4_torque_thrust_setpoint_node.cpp px4_msgs)
target_link_libraries(px4_torque_thrust_setpoint_node Eigen3::Eigen)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

install(FILES
  launch/mpc_offboard.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
