cmake_minimum_required(VERSION 3.5)
project(sharework_cembre_description)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ur_description REQUIRED)
# find_package(realsense2_description REQUIRED)
# find_package(robotiq_2f_85_gripper_visualization REQUIRED)

# Create an interface library target
add_library(${PROJECT_NAME} INTERFACE)

# Link the required components to the target
target_link_libraries(${PROJECT_NAME} INTERFACE
  ur_description::ur_description
#   realsense2_description::realsense2_description
#   robotiq_2f_85_gripper_visualization::robotiq_2f_85_gripper_visualization
)

# Include directories for the target
target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ament package
ament_package()