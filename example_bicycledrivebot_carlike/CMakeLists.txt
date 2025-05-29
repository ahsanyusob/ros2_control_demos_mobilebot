cmake_minimum_required(VERSION 3.16)
project(ros2_control_demo_bicycledrivebot_carlike LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# we want C++17 features
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

## COMPILE
add_library(
  ros2_control_demo_bicycledrivebot_carlike
  SHARED
  hardware/carlikebot_system.cpp
  hardware/carlikebot_actuator_over_socket.cpp
  hardware/carlikebot_sensor_over_socket.cpp
)
target_compile_features(ros2_control_demo_bicycledrivebot_carlike PUBLIC cxx_std_17)
target_include_directories(ros2_control_demo_bicycledrivebot_carlike PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/ros2_control_demo_bicycledrivebot_carlike>
)
target_link_libraries(ros2_control_demo_bicycledrivebot_carlike PUBLIC
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface ros2_control_demo_bicycledrivebot_carlike.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/ros2_control_demo_bicycledrivebot_carlike
)
install(
  DIRECTORY description/launch description/ros2_control description/urdf
  DESTINATION share/ros2_control_demo_bicycledrivebot_carlike
)
install(
  DIRECTORY bringup/launch bringup/config
  DESTINATION share/ros2_control_demo_bicycledrivebot_carlike
)
install(TARGETS ros2_control_demo_bicycledrivebot_carlike
  EXPORT export_ros2_control_demo_bicycledrivebot_carlike
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(bicycledrivebot_carlike_urdf_xacro test/test_urdf_xacro.py)
  ament_add_pytest_test(view_bicycledrivebot_carlike_launch test/test_view_robot_launch.py)
  ament_add_pytest_test(run_bicycledrivebot_carlike_launch test/test_carlikebot_launch.py)
  ament_add_pytest_test(run_bicycledrivebot_carlike_launch_remapped test/test_carlikebot_launch_remapped.py)
endif()

## EXPORTS
ament_export_targets(export_ros2_control_demo_bicycledrivebot_carlike HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
