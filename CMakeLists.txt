cmake_minimum_required(VERSION 3.5)
project(state_saver)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(EXPORT_COMPILE_COMMANDS ON)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(state_saver src/main.cpp
    src/SensorDataConfig.hpp
	src/SensorDataConfig.cpp
    )

ament_add_gtest(state_saver_test
  test/test_state_saver.cpp
  src/SensorDataConfig.hpp
  src/SensorDataConfig.cpp
)
add_executable(state_saver_checker
src/StateSaverChecker/main.cpp
)
ament_target_dependencies(state_saver rclcpp std_msgs sensor_msgs  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs)

ament_target_dependencies(state_saver_test rclcpp std_msgs sensor_msgs  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs)
  
ament_target_dependencies(state_saver_checker rclcpp std_msgs sensor_msgs  nav_msgs
  geometry_msgs
  tf2
  tf2_geometry_msgs)

install(TARGETS state_saver DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)
ament_package()



