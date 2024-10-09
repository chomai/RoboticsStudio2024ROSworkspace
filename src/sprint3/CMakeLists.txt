cmake_minimum_required(VERSION 3.8)
project(sprint3)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

# Create executables
add_executable(map_overlay_node src/map_overlay_node.cpp)
ament_target_dependencies(map_overlay_node rclcpp nav_msgs OpenCV)

add_executable(cylinder_detection_node src/cylinder_detection_node.cpp)
ament_target_dependencies(cylinder_detection_node 
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  visualization_msgs 
  tf2_geometry_msgs
  nav_msgs # Ensure nav_msgs is included here
  gazebo_ros
  OpenCV
)

# Install the executables
install(TARGETS
  map_overlay_node
  cylinder_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

