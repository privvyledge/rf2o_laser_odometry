cmake_minimum_required(VERSION 3.5)
project(rf2o_laser_odometry)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(class_loader REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2  REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
)

include_directories(  
  ${Boost_INCLUDE_DIRS}
)

set(DEPENDENCIES
  "rclcpp"
  "rclcpp_components"
  "class_loader"
  "std_msgs"
  "geometry_msgs"
  "tf2_geometry_msgs"
  "tf2"
  "tf2_ros"
  "sensor_msgs"
  "nav_msgs"
)

## Declare the component library
add_library(${PROJECT_NAME}_component SHARED
  src/CLaserOdometry2DNode.cpp
  src/CLaserOdometry2D.cpp
)
target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}_component ${DEPENDENCIES})

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rf2o::CLaserOdometry2DNode"
  EXECUTABLE rf2o_laser_odometry_node
)

## Install
install(TARGETS ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch  
  DESTINATION share/${PROJECT_NAME})

ament_package()
