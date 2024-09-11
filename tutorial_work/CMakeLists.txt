cmake_minimum_required(VERSION 3.8)
project(tutorial_work)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(OpenCV REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# Add Executable State_Estimation
add_executable(state_estimation src/state_estimation.cpp)
ament_target_dependencies(state_estimation 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs
)

# Add Executable Scan_To_Image
add_executable(scan_to_image src/scan_to_image.cpp)
ament_target_dependencies(scan_to_image 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  tf2 
  tf2_geometry_msgs
  OpenCV
)



# Create Install Targets
INSTALL(TARGETS
  state_estimation
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()