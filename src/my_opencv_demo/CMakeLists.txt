cmake_minimum_required(VERSION 3.8)
project(my_opencv_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Existing node
add_executable(minimal_opencv_ros2_node src/minimal_opencv_ros2_node.cpp)
ament_target_dependencies(minimal_opencv_ros2_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

# New image processor node LAB 1 DRAW CIRCLE IN MIDDLE
#add_executable(image_processor src/image_processor.cpp)
#ament_target_dependencies(image_processor rclcpp sensor_msgs cv_bridge image_transport OpenCV)

# New scan processor node LAB 2
#add_executable(scan_processor src/scan_processor.cpp)
#ament_target_dependencies(scan_processor rclcpp sensor_msgs)

# New Scantoimage node LAB 5
add_executable(scan_to_image_node src/scan_to_image_node.cpp)
ament_target_dependencies(scan_to_image_node rclcpp std_msgs sensor_msgs cv_bridge image_transport OpenCV)

# New simpleLocalizer SPRINT2
add_executable(simple_localizer_node src/simple_localizer_node.cpp)
ament_target_dependencies(simple_localizer_node 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  nav_msgs 
  OpenCV
  tf2
  tf2_geometry_msgs
)

# Declare the executable SPRINT2V2
add_executable(scan_matching_localizer src/scan_matching_localizer.cpp)

# Link against libraries
ament_target_dependencies(scan_matching_localizer
  rclcpp
  sensor_msgs
  nav_msgs
  cv_bridge
  OpenCV
  tf2
  tf2_ros
)

# Create an executable for the node
add_executable(map_laser_matching_node src/map_laser_matching_node.cpp)

# Link libraries
ament_target_dependencies(map_laser_matching_node
  rclcpp
  sensor_msgs
  nav_msgs
  OpenCV
  tf2
  tf2_geometry_msgs
)

target_link_libraries(scan_matching_localizer
  ${OpenCV_LIBRARIES}
)

install(TARGETS
  minimal_opencv_ros2_node
  #image_processor  # Include the new executable here
  #scan_processor
  scan_to_image_node
  #simple_localizer_node
  scan_matching_localizer
  map_laser_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

