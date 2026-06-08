cmake_minimum_required(VERSION 3.8)
project(conveyor_sort)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(xacro REQUIRED)

# Custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/ObjectPosition.msg"
  "msg/JamAlert.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

include_directories(include)

# Helper libraries
add_library(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp)

add_library(jam_detector src/jam_detector.cpp)
ament_target_dependencies(jam_detector rclcpp std_msgs)

# Common dependencies list
set(NODE_DEPS
  rclcpp std_msgs sensor_msgs geometry_msgs
  visualization_msgs cv_bridge image_transport OpenCV
)

# camera_node
add_executable(camera_node src/camera_node.cpp)
ament_target_dependencies(camera_node ${NODE_DEPS})
rosidl_target_interfaces(camera_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# vision_node
add_executable(vision_node src/vision_node.cpp)
ament_target_dependencies(vision_node ${NODE_DEPS})
rosidl_target_interfaces(vision_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# object_tracker_node
add_executable(object_tracker_node src/object_tracker_node.cpp)
ament_target_dependencies(object_tracker_node ${NODE_DEPS})
rosidl_target_interfaces(object_tracker_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# controller_node
add_executable(controller_node src/controller_node.cpp)
target_include_directories(controller_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(controller_node pid_controller)
ament_target_dependencies(controller_node ${NODE_DEPS})
rosidl_target_interfaces(controller_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# conveyor_node
add_executable(conveyor_node src/conveyor_node.cpp)
target_include_directories(conveyor_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(conveyor_node pid_controller)
ament_target_dependencies(conveyor_node ${NODE_DEPS})
rosidl_target_interfaces(conveyor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# sorting_node
add_executable(sorting_node src/sorting_node.cpp)
ament_target_dependencies(sorting_node ${NODE_DEPS})
rosidl_target_interfaces(sorting_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# jam_monitor_node
add_executable(jam_monitor_node src/jam_monitor_node.cpp)
target_include_directories(jam_monitor_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(jam_monitor_node jam_detector)
ament_target_dependencies(jam_monitor_node ${NODE_DEPS})
rosidl_target_interfaces(jam_monitor_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# rviz_display_node
add_executable(rviz_display_node src/rviz_display_node.cpp)
ament_target_dependencies(rviz_display_node ${NODE_DEPS})
rosidl_target_interfaces(rviz_display_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Install libraries
install(TARGETS
  pid_controller
  jam_detector
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Install executables
install(TARGETS
  camera_node
  vision_node
  object_tracker_node
  controller_node
  conveyor_node
  sorting_node
  jam_monitor_node
  rviz_display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz worlds models urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()