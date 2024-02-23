cmake_minimum_required(VERSION 3.8)
project(pointcloud_accumulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED) # Make sure to find OpenCV, needed by cv_bridge

add_executable(accumulator src/node.cpp)

# Include directories for the target
ament_target_dependencies(
  accumulator
  rclcpp
  sensor_msgs
  pcl_ros
  pcl_conversions
  cv_bridge
  image_transport
  OpenCV # Add OpenCV here
)

install(TARGETS
  accumulator
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
