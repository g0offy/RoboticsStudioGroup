cmake_minimum_required(VERSION 3.8)
project(week6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${nav_msgs_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# Add executable
add_executable(scan_matching_localiser src/scan_matching_localiser.cpp)

# Link libraries
target_link_libraries(scan_matching_localiser
  ${rclcpp_LIBRARIES}
  ${nav_msgs_LIBRARIES}
  ${sensor_msgs_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Install executable
install(TARGETS scan_matching_localiser
  DESTINATION lib/${PROJECT_NAME})

# Ament package
ament_package()