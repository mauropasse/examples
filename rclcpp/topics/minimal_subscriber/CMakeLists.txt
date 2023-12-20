cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)
include(${PROJECT_SOURCE_DIR}/cmake/build_types.cmake)

# Default to C++14
set(CMAKE_CXX_STANDARD 17)

#///////////////////////////////////

# Include build types file

set(CMAKE_VERBOSE_MAKEFILE ON)

# find dependencies


#///////////////////////////////////


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(RapidJSON REQUIRED)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp)

add_executable(subscriber_lambda lambda.cpp)
ament_target_dependencies(subscriber_lambda rclcpp std_msgs)

if(TARGET rapidjson::rapidjson)
  target_link_libraries(subscriber_lambda rapidjson::rapidjson)
else()
  target_include_directories(subscriber_lambda PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
endif()

install(TARGETS
  subscriber_lambda
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
