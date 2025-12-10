cmake_minimum_required(VERSION 3.8)
project(sensor_simulator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Create executable
add_executable(transform_sync src/transform_sync.cpp)
ament_target_dependencies(transform_sync 
  rclcpp 
  std_msgs 
  geometry_msgs 
  nav_msgs 
  sensor_msgs 
  tf2 
  tf2_ros
  builtin_interfaces)

# Install executables
install(TARGETS
  transform_sync
  DESTINATION lib/${PROJECT_NAME})

# Install other files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()