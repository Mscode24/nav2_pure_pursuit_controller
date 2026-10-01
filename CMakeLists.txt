cmake_minimum_required(VERSION 3.8)
project(nav2_pure_pursuit_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -std=c++17)
endif()
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


add_library(nav2_pure_pursuit_controller SHARED
  src/pure_pursuit_controller.cpp
)

target_include_directories(nav2_pure_pursuit_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)



ament_target_dependencies(nav2_pure_pursuit_controller
  rclcpp
  rclcpp_lifecycle
  nav2_core
  pluginlib
  geometry_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

# Register plugin description file with pluginlib
pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS nav2_pure_pursuit_controller
  DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()