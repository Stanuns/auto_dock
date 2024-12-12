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

ament_package()
