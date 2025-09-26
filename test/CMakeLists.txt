cmake_minimum_required(VERSION 3.5)
project(ros2_persistent_parameter_server_test VERSION 1.0.1)

# Set Release build if no build type was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
      "Build type for the build. Possible values are: Debug, Release, RelWithDebInfo, MinSizeRel"
      FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
      "Debug" "Release" "RelWithDebInfo" "MinSizeRel")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# Enable additional warnings and warnings as errors
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)

add_executable(client_default
  src/test_default.cpp
  src/persist_parameter_client.cpp
)

target_link_libraries(client_default
  PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${std_msgs_TARGETS}
)

target_include_directories(client_default
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

add_executable(client_with_node_options
  src/test_with_node_options.cpp
  src/persist_parameter_client.cpp
)

target_link_libraries(client_with_node_options
  PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${std_msgs_TARGETS}
)

target_include_directories(client_with_node_options
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

install(TARGETS client_default client_with_node_options DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
