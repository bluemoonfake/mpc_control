cmake_minimum_required(VERSION 3.16)

project(mpc_control_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(message_files
  "msg/TrajectoryPoint.msg"
  "msg/ReferenceTrajectory.msg"
  "msg/TrajectoryCommand.msg"
  "msg/VehicleState.msg"
  "msg/PredictedTrajectory.msg"
  "msg/MpcDiagnostics.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${message_files}
  DEPENDENCIES
    builtin_interfaces
    std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_message_schema
    test/test_message_schema.py
  )
endif()

ament_package()
