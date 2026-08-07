cmake_minimum_required(VERSION 3.16)

project(mpc_control_px4)

find_package(ament_cmake REQUIRED)
find_package(mpc_control_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(px4_trajectory_adapter
  src/px4_trajectory_adapter.cpp
)

target_include_directories(px4_trajectory_adapter
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(px4_trajectory_adapter PUBLIC cxx_std_20)

ament_target_dependencies(px4_trajectory_adapter
  mpc_control_msgs
  px4_msgs
)

add_library(px4_trajectory_publisher
  src/px4_estimator_validity_monitor.cpp
  src/px4_timestamp_source.cpp
  src/px4_trajectory_setpoint_publisher.cpp
)

target_include_directories(px4_trajectory_publisher
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(px4_trajectory_publisher PUBLIC cxx_std_20)

target_link_libraries(px4_trajectory_publisher
  px4_trajectory_adapter
)

ament_target_dependencies(px4_trajectory_publisher
  mpc_control_msgs
  px4_msgs
  rclcpp
)

add_executable(px4_trajectory_setpoint_publisher
  src/main.cpp
)

target_link_libraries(px4_trajectory_setpoint_publisher
  px4_trajectory_publisher
)

ament_target_dependencies(px4_trajectory_setpoint_publisher
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS px4_trajectory_adapter
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  TARGETS px4_trajectory_publisher px4_trajectory_setpoint_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(px4_trajectory_adapter_test
    test/px4_trajectory_adapter_test.cpp
  )
  if(TARGET px4_trajectory_adapter_test)
    target_link_libraries(px4_trajectory_adapter_test px4_trajectory_adapter)
    ament_target_dependencies(px4_trajectory_adapter_test
      mpc_control_msgs
      px4_msgs
    )
  endif()

  ament_add_gtest(px4_trajectory_setpoint_publisher_test
    test/px4_trajectory_setpoint_publisher_test.cpp
  )
  if(TARGET px4_trajectory_setpoint_publisher_test)
    target_link_libraries(px4_trajectory_setpoint_publisher_test
      px4_trajectory_publisher
    )
    ament_target_dependencies(px4_trajectory_setpoint_publisher_test
      mpc_control_msgs
      px4_msgs
      rclcpp
    )
  endif()

  ament_add_gtest(px4_estimator_validity_monitor_test
    test/px4_estimator_validity_monitor_test.cpp
  )
  if(TARGET px4_estimator_validity_monitor_test)
    target_link_libraries(px4_estimator_validity_monitor_test
      px4_trajectory_publisher
    )
    ament_target_dependencies(px4_estimator_validity_monitor_test
      px4_msgs
    )
  endif()

  ament_add_gtest(px4_timestamp_source_test
    test/px4_timestamp_source_test.cpp
  )
  if(TARGET px4_timestamp_source_test)
    target_link_libraries(px4_timestamp_source_test
      px4_trajectory_publisher
    )
    ament_target_dependencies(px4_timestamp_source_test
      px4_msgs
    )
  endif()
endif()

ament_export_dependencies(
  mpc_control_msgs
  px4_msgs
  rclcpp
)

ament_package()
