cmake_minimum_required(VERSION 3.8)
project(custom_rover_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(isaac_ros_tensor_list_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_library(depth_decoder_node SHARED
  src/depth_decoder_node.cpp
)
add_library(PedestrianFollower SHARED
  src/pedestrian_follower.cpp # Make sure this file actually exists!
)
target_include_directories(depth_decoder_node PRIVATE include)
ament_target_dependencies(depth_decoder_node
  rclcpp
  rclcpp_components
  sensor_msgs
  cv_bridge
  OpenCV
  isaac_ros_tensor_list_interfaces
  image_transport
  
)
target_include_directories(PedestrianFollower PRIVATE include)
ament_target_dependencies(PedestrianFollower
  rclcpp
  rclcpp_components
  sensor_msgs
  vision_msgs 
  cv_bridge
  OpenCV
  isaac_ros_tensor_list_interfaces
  image_transport
  message_filters
  std_msgs
  geometry_msgs 
)
rclcpp_components_register_node(depth_decoder_node
  PLUGIN "custom_rover_nodes::DepthDecoderNode"
  EXECUTABLE depth_decoder_node_exe
)
rclcpp_components_register_node(PedestrianFollower
  PLUGIN "custom_rover_nodes::PedestrianFollower"
  EXECUTABLE PedestrianFollower_exe
)
install(TARGETS
  depth_decoder_node
  PedestrianFollower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(PROGRAMS
  src/rover_control_node.py
  src/rover_pedestrian_control.py
  src/pedestrian_decoder_node.py
  DESTINATION lib/${PROJECT_NAME}
)
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
