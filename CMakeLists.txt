cmake_minimum_required(VERSION 3.5)
project(mobile_robot_ai)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(data_subscriber src/data_subscriber.cpp)
ament_target_dependencies(data_subscriber rclcpp sensor_msgs)

install(DIRECTORY 
  launch 
  maps 
  rviz 
  worlds 
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  data_subscriber
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
