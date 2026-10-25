cmake_minimum_required(VERSION 3.8)
project(sp_controller_server)
# 轨迹计算使用 C++17 的 std::clamp
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
# Eigen 提供预测矩阵运算和二次规划所需的LDLT分解
find_package(Eigen3 REQUIRED)
include_directories(include)
add_library(controller_server_core SHARED
  src/controller_server.cpp
)
target_include_directories(controller_server_core PRIVATE include)
ament_target_dependencies(controller_server_core
  rclcpp
  nav_msgs
  geometry_msgs
  pluginlib
  std_srvs
  tf2_ros
  tf2_geometry_msgs
)
# 控制插件库编译MpcController，由pluginlib按注册名加载
add_library(controller_plugins SHARED
  plugins/mpc_controller.cpp
)
# 为MPC及其矩阵求解头文件提供Eigen依赖
target_link_libraries(controller_plugins Eigen3::Eigen)
target_include_directories(controller_plugins PRIVATE include)
ament_target_dependencies(controller_plugins
  rclcpp
  nav_msgs
  geometry_msgs
  pluginlib
  tf2_ros
  tf2_geometry_msgs
)
pluginlib_export_plugin_description_file(sp_controller_server plugin_description.xml)
add_executable(controller_node src/controller_node.cpp)
target_link_libraries(controller_node controller_server_core)
ament_target_dependencies(controller_node rclcpp)
install(TARGETS
  controller_server_core
  controller_plugins
  DESTINATION lib
)
install(TARGETS
  controller_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME})
install(FILES plugin_description.xml DESTINATION share/${PROJECT_NAME})
ament_package()
