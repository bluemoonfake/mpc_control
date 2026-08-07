cmake_minimum_required(VERSION 3.16)

project(mpc_control_ros)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(eigen3_cmake_module REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(mpc_control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# The standalone core is intentionally not an ament package yet. Build it as
# an internal dependency so this wrapper cannot accidentally acquire ROS/PX4
# dependencies in the control mathematics.
set(MPC_CONTROL_BUILD_TESTS OFF CACHE BOOL "Build standalone mpc_control tests" FORCE)
add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../.."
  "${CMAKE_CURRENT_BINARY_DIR}/mpc_control_core"
  EXCLUDE_FROM_ALL
)

add_library(mpc_control_ros_node
  src/mpc_trajectory_node.cpp
)

target_include_directories(mpc_control_ros_node
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_compile_features(mpc_control_ros_node PUBLIC cxx_std_20)

ament_target_dependencies(mpc_control_ros_node
  lifecycle_msgs
  mpc_control_msgs
  rclcpp
  rclcpp_lifecycle
)

target_link_libraries(mpc_control_ros_node
  Eigen3::Eigen
  mpc_core
)

add_executable(mpc_trajectory_node
  src/main.cpp
)

target_link_libraries(mpc_trajectory_node mpc_control_ros_node)

ament_target_dependencies(mpc_trajectory_node
  rclcpp
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS mpc_control_ros_node mpc_trajectory_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(rosgraph_msgs REQUIRED)
  ament_add_gtest(mpc_control_ros_test
    test/mpc_trajectory_node_test.cpp
  )
  if(TARGET mpc_control_ros_test)
    target_link_libraries(mpc_control_ros_test mpc_control_ros_node)
    ament_target_dependencies(mpc_control_ros_test
      lifecycle_msgs
      mpc_control_msgs
      rclcpp
      rclcpp_lifecycle
      rosgraph_msgs
    )
  endif()

  add_launch_test(
    test/ros_wrapper_launch_test.py
    TARGET ros_wrapper_launch_test
    TIMEOUT 90
  )

  # add_launch_test does not propagate the isolated install environment of
  # ament dependencies. The launch test imports generated Python messages and
  # starts the installed wrapper executable, so make those paths explicit and
  # keep the test reproducible from a plain `colcon test` invocation.
  get_filename_component(_ros_install_root "${CMAKE_INSTALL_PREFIX}" DIRECTORY)
  set(_mpc_msgs_prefix "${_ros_install_root}/mpc_control_msgs")
  set(_mpc_msgs_pythonpath
    "${_mpc_msgs_prefix}/lib/python${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}/site-packages")
  set(_launch_test_pythonpath "${_mpc_msgs_pythonpath}")
  if(DEFINED ENV{PYTHONPATH} AND NOT "$ENV{PYTHONPATH}" STREQUAL "")
    string(APPEND _launch_test_pythonpath ":$ENV{PYTHONPATH}")
  endif()
  set(_launch_test_ament_prefix
    "${CMAKE_INSTALL_PREFIX};${_mpc_msgs_prefix}")
  if(DEFINED ENV{AMENT_PREFIX_PATH} AND NOT "$ENV{AMENT_PREFIX_PATH}" STREQUAL "")
    string(APPEND _launch_test_ament_prefix ":$ENV{AMENT_PREFIX_PATH}")
  endif()
  set_tests_properties(ros_wrapper_launch_test PROPERTIES
    ENVIRONMENT
      "PYTHONPATH=${_launch_test_pythonpath};AMENT_PREFIX_PATH=${_launch_test_ament_prefix};LD_LIBRARY_PATH=${_mpc_msgs_prefix}/lib:$ENV{LD_LIBRARY_PATH}"
  )
endif()

ament_export_dependencies(
  eigen3_cmake_module
  lifecycle_msgs
  mpc_control_msgs
  rclcpp
  rclcpp_lifecycle
)

ament_package()
