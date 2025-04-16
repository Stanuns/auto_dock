cmake_minimum_required(VERSION 3.8)
project(auto_dock)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 3.3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

add_executable(lidar_align src/lidar_align.cpp)
ament_target_dependencies(
  lidar_align
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  tf2 
  tf2_ros 
  tf2_geometry_msgs
)
target_link_libraries(lidar_align Eigen3::Eigen)

target_include_directories(lidar_align PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_align PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(lidar_align_simple src/lidar_align_simple.cpp)
ament_target_dependencies(
  lidar_align_simple
  sensor_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  robot_interfaces
  tf2 
  tf2_ros 
  tf2_geometry_msgs
  visualization_msgs
)
target_link_libraries(lidar_align_simple Eigen3::Eigen)

target_include_directories(lidar_align_simple PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(lidar_align_simple PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

add_executable(wall_detection src/wall_detection.cpp)
ament_target_dependencies(wall_detection 
  rclcpp 
  sensor_msgs 
  geometry_msgs 
  pcl_conversions 
  PCL
  tf2_ros
  tf2
  tf2_geometry_msgs
  robot_interfaces
)
target_include_directories(wall_detection PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(wall_detection PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

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

install(TARGETS
  lidar_align
  lidar_align_simple 
  wall_detection
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME})


# add_library(action_server SHARED
#   src/auto_dock_server.cpp src/dock_driver.cpp src/dock_driver_states.cpp)
# target_include_directories(action_server PRIVATE
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_definitions(action_server
#   PRIVATE "AUTO_DOCK_BUILDING_DLL")
# ament_target_dependencies(action_server
#   "robot_interfaces"
#   "rclcpp"
#   "rclcpp_action"
#   "rclcpp_components"
#   "message_filters"
#   "nav_msgs"
#   "geometry_msgs"
#   "sensor_msgs"
#   "tf2" 
#   "tf2_ros" 
#   "tf2_geometry_msgs")
# rclcpp_components_register_node(action_server PLUGIN "auto_dock::AutoDockActionServer" EXECUTABLE auto_dock_action_server)
# install(TARGETS
#   action_server
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin)
include_directories(include)
add_executable(auto_dock_action_server src/auto_dock_server.cpp src/dock_driver.cpp src/dock_driver_states.cpp)
ament_target_dependencies(auto_dock_action_server
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  message_filters
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)
install(TARGETS auto_dock_action_server
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(remote_auto_dock src/remote_auto_dock.cpp)
ament_target_dependencies(remote_auto_dock
  robot_interfaces
  rclcpp
  rclcpp_action
  rclcpp_components
  message_filters
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
)
install(TARGETS remote_auto_dock
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(remote_auto_dock_hm src/remote_auto_dock_hm.cpp)
ament_target_dependencies(remote_auto_dock_hm
  rclcpp
  rclcpp_action
  rclcpp_components
  message_filters
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  std_msgs
)
install(TARGETS remote_auto_dock_hm
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(nav_to_goal_recycle src/nav_to_goal_recycle.cpp)
ament_target_dependencies(nav_to_goal_recycle
  rclcpp
  rclcpp_action
  rclcpp_components
  message_filters
  nav_msgs
  geometry_msgs
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_msgs
  std_msgs
)
install(TARGETS nav_to_goal_recycle
    DESTINATION lib/${PROJECT_NAME}
)

add_library(dock_action_client SHARED #if use 'action_client', ‘ros2 run auto_dock auto_dock_action_client’ cannot run successfully.
  src/auto_dock_client.cpp)
target_include_directories(dock_action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(dock_action_client
  PRIVATE "AUTO_DOCK_BUILDING_DLL")
ament_target_dependencies(dock_action_client
  "robot_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(dock_action_client PLUGIN "auto_dock::AutoDockActionClient" EXECUTABLE auto_dock_action_client)
install(TARGETS
  dock_action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)


ament_package()
