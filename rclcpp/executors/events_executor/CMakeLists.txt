cmake_minimum_required(VERSION 3.5)
project(events_executor)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(irobot_events_executor REQUIRED)

add_executable(events_executor events_executor.cpp)
ament_target_dependencies(events_executor irobot_events_executor rclcpp)

install(TARGETS
  events_executor
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
