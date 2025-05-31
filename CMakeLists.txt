cmake_minimum_required(VERSION 3.5)
project(state_saver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(state_saver src/main.cpp src/StateSaver.cpp)
ament_target_dependencies(state_saver rclcpp std_msgs sensor_msgs)

install(TARGETS state_saver DESTINATION lib/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test
    test/test_state_saver.cpp
    src/StateSaver.cpp
  )
  
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  
  ament_target_dependencies(${PROJECT_NAME}_test
    rclcpp
    std_msgs
    sensor_msgs
  )
endif()
