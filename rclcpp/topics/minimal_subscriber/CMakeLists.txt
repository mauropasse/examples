cmake_minimum_required(VERSION 3.5)
project(examples_rclcpp_minimal_subscriber)

# Include build types file
include(${PROJECT_SOURCE_DIR}/cmake/build_types.cmake)

set(CMAKE_VERBOSE_MAKEFILE ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Boost REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(irobot2_gazebo_msgs REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(RapidJSON CONFIG REQUIRED)

# ament_export_include_directories(include)
# ament_export_dependencies(rclcpp)

# # Search OpenSSL
# find_package(PkgConfig REQUIRED)
# pkg_search_module(OPENSSL REQUIRED openssl)

# if( OPENSSL_FOUND )
#     include_directories(${OPENSSL_INCLUDE_DIRS})
#     message(STATUS "Using OpenSSL ${OPENSSL_VERSION}")
# else()
#     # Error; with REQUIRED, pkg_search_module() will throw an error by it's own
# endif()

# target_link_libraries(${YOUR_TARGET_HERE} ${OPENSSL_LIBRARIES})

# get_cmake_property(_variableNames VARIABLES)
# list (SORT _variableNames)
# foreach (_variableName ${_variableNames})
#     message(STATUS "${_variableName}=${${_variableName}}")
# endforeach()

# message(STATUS "OPENSSL_LIBRARIES = ${OPENSSL_LIBRARIES}")

# include_directories(${OPENSSL_INCLUDE_DIR})

# # Message Converter
# add_library(message_converter SHARED
#   src/message_converter.cpp
# )

# # Note: The ament_target_dependencies below doesn't seem
# # to be working for robot2_gazebo_msgs, so we manually set
# # the include and shared lib dirs
# target_include_directories(message_converter PUBLIC
#   include
#   ${irobot2_gazebo_msgs_INCLUDES_RELEASE}
# )

# ament_target_dependencies(message_converter
#   "geometry_msgs"
#   "rclcpp"
#   "rosgraph_msgs"
#   "sensor_msgs"
#   "std_msgs"
# )

# target_link_libraries(message_converter
#   ${OPENSSL_LIBRARIES}
#   ${Boost_LIBRARIES}
#   ${irobot2_gazebo_msgs_LIBRARIES_RELEASE}
# )
# # If rapidjson is being provided by conan, then we can use modern cmake and link in the target
# # rapidjson::rapidjson, otherwise we fallback to the old way and just expose the headers.
# # The conan version package exposes a similar variable so we need this logic anyways.
# if(TARGET rapidjson::rapidjson)
#   target_link_libraries(message_converter rapidjson::rapidjson)
# else()
#   target_include_directories(message_converter PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
# endif()

# ament_export_libraries(message_converter)

# # iRobot Bridge
# add_library(irobot_bridge SHARED
#   src/irobot_bridge.cpp
# )
# target_link_libraries(irobot_bridge ${OPENSSL_LIBRARIES})
# target_link_libraries(irobot_bridge message_converter rclcpp::rclcpp  ${Boost_LIBRARIES})

# # iRobot Bridge Node
# add_executable(irobot_bridge_node src/irobot_bridge_node.cpp)
# target_link_libraries(irobot_bridge_node irobot_bridge rclcpp::rclcpp)

# if(BUILD_TESTING)
#   find_package(ament_cmake_xmllint REQUIRED)
#   find_package(ament_cmake_cppcheck REQUIRED)
#   find_package(ament_cmake_flake8 REQUIRED)
#   find_package(ament_cmake_pep257 REQUIRED)
#   find_package(ament_index_cpp REQUIRED)
#   ament_xmllint()
#   ament_cppcheck(LANGUAGE "c++")
#   ament_flake8()
#   ament_pep257()
#   # TODO: Move simple-web-socket-server outside the package and enable
#   # ament_cmake_lint_cmake
#   # https://irobot.atlassian.net/browse/DPSIM-3499
#   # https://irobot.atlassian.net/browse/DPSIM-3500
#   # find_package(ament_cmake_lint_cmake REQUIRED)
#   # ament_lint_cmake()
#   # TODO: Enable linting tests after fix possible errors
#   # find_package(ament_lint_auto REQUIRED)
#   # # the following line skips the linter which checks for copyrights
#   # # uncomment the line when a copyright and license is not present in all source files
#   # set(ament_cmake_copyright_FOUND TRUE)
#   # # the following line skips cpplint (only works in a git repo)
#   # # uncomment the line when this package is not in a git repo
#   # set(ament_cmake_cpplint_FOUND TRUE)
#   # ament_lint_auto_find_test_dependencies()

#   # Message Converter Test
#   # find_package(ament_cmake_gtest REQUIRED)
#   # ament_add_gtest(message_converter_test
#   #   test/message_converter_test.cpp
#   # )

#   # target_link_libraries(message_converter_test message_converter ament_index_cpp::ament_index_cpp)
#   # # If rapidjson is being provided by conan, then we can use modern cmake and link in the target
#   # # rapidjson::rapidjson, otherwise we fallback to the old way and just expose the headers.
#   # # The conan version package exposes a similar variable so we need this logic anyways.
#   # if(TARGET rapidjson::rapidjson)
#   #   target_link_libraries(message_converter_test rapidjson::rapidjson)
#   # else()
#   #   target_include_directories(message_converter_test PUBLIC ${RAPIDJSON_INCLUDE_DIRS})
#   # endif()

#   # ament_add_gtest(irobot_rosbridge_test
#   #   test/irobot_rosbridge_test.cpp
#   # )
#   # ament_target_dependencies(irobot_rosbridge_test
#   #   "rclcpp"
#   #   "std_msgs"
#   # )
#   # target_link_libraries(irobot_rosbridge_test irobot_bridge ${OPENSSL_LIBRARIES})
# endif()

# # Installation setup
# install(DIRECTORY include
#         DESTINATION include/${PROJECT_NAME})

# install(DIRECTORY
#         launch
#         test
#         DESTINATION share/${PROJECT_NAME})

# install(TARGETS
#         irobot_bridge
#         irobot_bridge_node
#         message_converter
#         ARCHIVE DESTINATION lib
#         LIBRARY DESTINATION lib
#         RUNTIME DESTINATION lib/${PROJECT_NAME})

# ament_package()
