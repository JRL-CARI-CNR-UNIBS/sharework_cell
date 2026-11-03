cmake_minimum_required(VERSION 3.16)
project(sharework_gazebo)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
# gz_ros2_control links gz-sim8 without finding it.
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pinocchio REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(sharework_gazebo_hardware SHARED src/gravity_compensated_gz_system.cpp)
target_include_directories(sharework_gazebo_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  # gz_hardware_plugins carries no include directory, and the headers sit one level down.
  ${gz_ros2_control_DIR}/../../../include/gz_ros2_control)
target_link_libraries(sharework_gazebo_hardware PUBLIC
  gz_ros2_control::gz_hardware_plugins
  hardware_interface::hardware_interface
  pinocchio::pinocchio
  pluginlib::pluginlib
  rclcpp::rclcpp)

add_executable(points_frame_relay src/points_frame_relay.cpp)
target_link_libraries(points_frame_relay rclcpp::rclcpp ${sensor_msgs_TARGETS})

# gz_ros2_control loads hardware plugins of this base class.
pluginlib_export_plugin_description_file(gz_ros2_control gz_hardware_plugins.xml)

install(TARGETS sharework_gazebo_hardware
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)
install(TARGETS points_frame_relay DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config launch urdf worlds DESTINATION share/${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(gz_ros2_control hardware_interface pinocchio pluginlib rclcpp)
ament_package()
