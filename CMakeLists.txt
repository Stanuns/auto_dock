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
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)

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
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch
   DESTINATION share/${PROJECT_NAME})


add_library(action_server SHARED
  src/auto_dock_server.cpp src/dock_driver.cpp src/dock_driver_states.cpp)
target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_server
  PRIVATE "AUTO_DOCK_BUILDING_DLL")
ament_target_dependencies(action_server
  "robot_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "message_filters"
  "nav_msgs"
  "geometry_msgs"
  "sensor_msgs"
  "tf2" 
  "tf2_ros" 
  "tf2_geometry_msgs")
rclcpp_components_register_node(action_server PLUGIN "auto_dock::AutoDockActionServer" EXECUTABLE auto_dock_action_server)
install(TARGETS
  action_server
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

add_library(action_client SHARED
  src/auto_dock_client.cpp)
target_include_directories(action_client PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(action_client
  PRIVATE "ACTION_TUTORIALS_CPP_BUILDING_DLL")
ament_target_dependencies(action_client
  "robot_interfaces"
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components")
rclcpp_components_register_node(action_client PLUGIN "auto_dock::AutoDockActionClient" EXECUTABLE auto_dock_action_client)
install(TARGETS
  action_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
