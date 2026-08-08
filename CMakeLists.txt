cmake_minimum_required(VERSION 3.16)

project(mpc_controller)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/TrajectoryPoint.msg"
  "msg/ReferenceTrajectory.msg"
  "msg/VehicleState.msg"
  "msg/DirectAccelerationCommand.msg"
  "msg/MpcTranslationalOutput.msg"
  "msg/M3ControlOutput.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

function(add_mpc_node target source)
  add_executable(${target} ${source})
  target_compile_features(${target} PRIVATE cxx_std_17)
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${target} rclcpp px4_msgs)
  target_link_libraries(${target} Eigen3::Eigen)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${target} "${cpp_typesupport_target}")
  install(TARGETS ${target} DESTINATION lib/${PROJECT_NAME})
endfunction()

set(MPC_SOLVER_SOURCES
  "third_party/mrs_mpc_solvers/src/mpc_controller/mpc_controller_solver.cpp"
  "third_party/mrs_mpc_solvers/src/mpc_controller/solver/ldl.cpp"
  "third_party/mrs_mpc_solvers/src/mpc_controller/solver/matrix_support.cpp"
  "third_party/mrs_mpc_solvers/src/mpc_controller/solver/solver.cpp"
  "third_party/mrs_mpc_solvers/src/mpc_controller/solver/util.cpp"
)

add_library(mpc_translational_solver STATIC ${MPC_SOLVER_SOURCES})
target_compile_features(mpc_translational_solver PRIVATE cxx_std_20)
target_include_directories(mpc_translational_solver PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/third_party/mrs_mpc_solvers/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(mpc_translational_solver PUBLIC Eigen3::Eigen)

add_mpc_node(reference_generator_node src/reference_generator_node.cpp)
add_mpc_node(mpc_controller_node src/mpc_controller_node.cpp)
target_link_libraries(mpc_controller_node mpc_translational_solver)
add_mpc_node(vehicle_state_bridge_node src/vehicle_state_bridge_node.cpp)
add_mpc_node(px4_attitude_setpoint_node src/px4_attitude_setpoint_node.cpp)
add_mpc_node(px4_wrench_bridge_node src/px4_wrench_bridge_node.cpp)
add_mpc_node(m4_timing_probe test/m4_timing_probe/main.cpp)

add_executable(m5_hold_capture_test test/m5_hold_capture/main.cpp)
target_compile_features(m5_hold_capture_test PRIVATE cxx_std_17)
target_include_directories(m5_hold_capture_test PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)

install(PROGRAMS scripts/m5_safety_monitor.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

install(TARGETS mpc_translational_solver ARCHIVE DESTINATION lib)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
