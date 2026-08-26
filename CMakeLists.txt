cmake_minimum_required(VERSION 3.16)

project(mpc_controller)

include(CTest)

# The MPC hot path is Eigen-heavy and must run inside a 20 ms control period.
# Keep Release as the default while allowing an explicit build type to override it.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
set(MPC_ACADOS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party/acados" CACHE PATH
    "Local acados installation root")
set(MPC_ACADOS_GENERATED_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/build/acados_tpmc" CACHE PATH
    "Generated acados TMPC solver directory")
set(MPC_PX4_MSGS_CMAKE_DIR
    "${CMAKE_CURRENT_SOURCE_DIR}/install/px4_msgs/px4_msgs/share/px4_msgs/cmake"
    CACHE PATH "Workspace-local px4_msgs CMake package directory")
list(PREPEND CMAKE_PREFIX_PATH "${MPC_ACADOS_ROOT}")
find_package(acados CONFIG REQUIRED PATHS "${MPC_ACADOS_ROOT}/cmake"
             NO_DEFAULT_PATH)
find_package(px4_msgs CONFIG REQUIRED PATHS "${MPC_PX4_MSGS_CMAKE_DIR}"
             NO_DEFAULT_PATH)
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

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

if(NOT EXISTS "${MPC_ACADOS_GENERATED_DIR}/tpmc_generated_solver_bridge.hpp")
  message(FATAL_ERROR
    "Generated acados TMPC solver is missing. Run "
    "python3 tools/acados/generate_tpmc_solver.py first, then configure again.")
endif()

file(GLOB MPC_ACADOS_MODEL_SOURCES CONFIGURE_DEPENDS
  "${MPC_ACADOS_GENERATED_DIR}/tpmc_model/*.c")
file(GLOB MPC_ACADOS_CONSTRAINT_SOURCES CONFIGURE_DEPENDS
  "${MPC_ACADOS_GENERATED_DIR}/tpmc_constraints/*.c")
file(GLOB MPC_ACADOS_COST_SOURCES CONFIGURE_DEPENDS
  "${MPC_ACADOS_GENERATED_DIR}/tpmc_cost/*.c")
file(GLOB MPC_ACADOS_SOLVER_SOURCES CONFIGURE_DEPENDS
  "${MPC_ACADOS_GENERATED_DIR}/acados_solver_ocp_tpmc_*.c")
set(MPC_ACADOS_SOURCES
  ${MPC_ACADOS_MODEL_SOURCES}
  ${MPC_ACADOS_CONSTRAINT_SOURCES}
  ${MPC_ACADOS_COST_SOURCES}
  ${MPC_ACADOS_SOLVER_SOURCES})
if(NOT MPC_ACADOS_SOURCES)
  message(FATAL_ERROR "No generated acados TMPC C sources found")
endif()

add_library(acados_tpmc_generated STATIC ${MPC_ACADOS_SOURCES})
set_target_properties(acados_tpmc_generated PROPERTIES C_STANDARD 99)
target_compile_definitions(acados_tpmc_generated PRIVATE ACADOS_SILENT)
target_include_directories(acados_tpmc_generated PRIVATE
  "${MPC_ACADOS_GENERATED_DIR}")
target_link_libraries(acados_tpmc_generated PUBLIC acados)

add_library(mpc_core STATIC
  src/solver/tpmc_model.cpp
  src/solver/tpmc_constraints.cpp
  src/solver/tpmc_reference.cpp
  src/solver/tpmc_solver.cpp
  src/solver/acados_tpmc_solver.cpp)
target_compile_features(mpc_core PUBLIC cxx_std_17)
target_include_directories(mpc_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(mpc_core PRIVATE "${MPC_ACADOS_GENERATED_DIR}")
target_link_libraries(mpc_core PUBLIC acados_tpmc_generated)

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

add_mpc_node(reference_generator_node src/mission/reference_generator_node.cpp
  px4_msgs px4_ros2_cpp std_msgs std_srvs visualization_msgs)
add_mpc_node(mpc_controller_node src/controller/mpc_controller_node.cpp builtin_interfaces std_msgs)
target_link_libraries(mpc_controller_node mpc_core Eigen3::Eigen)
set_target_properties(mpc_controller_node PROPERTIES
  BUILD_RPATH "${MPC_ACADOS_ROOT}/lib"
  INSTALL_RPATH "$ORIGIN/..")
target_link_options(mpc_controller_node PRIVATE "-Wl,--disable-new-dtags")
add_mpc_node(vehicle_state_bridge_node src/bridge/vehicle_state_bridge_node.cpp px4_msgs)
add_mpc_node(collective_identification_node src/controller/collective_identification_node.cpp
  px4_msgs std_msgs std_srvs)
target_link_libraries(collective_identification_node Eigen3::Eigen)
add_mpc_node(px4_attitude_mode_node src/controller/px4_attitude_mode_node.cpp px4_msgs px4_ros2_cpp std_msgs std_srvs)
target_link_libraries(px4_attitude_mode_node Eigen3::Eigen px4_ros2_cpp::px4_ros2_cpp)

if(BUILD_TESTING)
  add_executable(tpmc_core_test test/cpp/tpmc_core_test.cpp)
  target_compile_features(tpmc_core_test PRIVATE cxx_std_17)
  target_include_directories(tpmc_core_test PRIVATE include)
  target_link_libraries(tpmc_core_test PRIVATE mpc_core Eigen3::Eigen)
  set_target_properties(tpmc_core_test PROPERTIES
    BUILD_RPATH "${MPC_ACADOS_ROOT}/lib")
  target_link_options(tpmc_core_test PRIVATE "-Wl,--disable-new-dtags")
  add_test(NAME tpmc_core_test COMMAND tpmc_core_test)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS mpc_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)
install(DIRECTORY "${MPC_ACADOS_ROOT}/lib/"
  DESTINATION lib
  FILES_MATCHING
  PATTERN "libacados.so*"
  PATTERN "libblasfeo.so*"
  PATTERN "libhpipm.so*"
  PATTERN "libqpOASES_e.so*")

install(FILES
  launch/mpc_offboard.launch.py
  launch/collective_identification.launch.py
  DESTINATION share/${PROJECT_NAME}/launch)

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
