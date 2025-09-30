cmake_minimum_required(VERSION 3.5)
project(persist_parameter_server VERSION 1.0.1)

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
find_package(std_srvs REQUIRED)
find_package(rmw REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(yaml_cpp_vendor REQUIRED)

option(CLIENT_TEST_DEMO "Build client test demo" OFF)

add_executable(server
 server/src/parameter_server.cpp
 server/src/main.cpp
)

# yaml-cpp updates CMake thing significantly on v0.8.0 or later.
# so we ended up having the if statement to process differently instead of creating branches.
# see https://github.com/jbeder/yaml-cpp/releases/tag/0.8.0
find_package(yaml-cpp 0.8.0 QUIET)
if (yaml-cpp_FOUND)
  message(STATUS "yaml-cpp package is greater equal than version 0.8.0")
  target_link_libraries(server
    rclcpp::rclcpp
    rclcpp_components::component
    rcutils::rcutils
    yaml-cpp::yaml-cpp
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${Boost_LIBRARIES}
  )
else()
  message(STATUS "yaml-cpp package is less than version 0.8.0")
  find_package(yaml-cpp REQUIRED)
  target_link_libraries(server
    rclcpp::rclcpp
    rclcpp_components::component
    rcutils::rcutils
    yaml-cpp
    ${std_msgs_TARGETS}
    ${std_srvs_TARGETS}
    ${Boost_LIBRARIES}
  )
endif()

target_include_directories(server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/server/include>
)

install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  server/launch
  server/param
  DESTINATION share/${PROJECT_NAME}/
)

# Build client test demo
if (CLIENT_TEST_DEMO)
add_executable(client_default
  test/src/test_default.cpp
  test/src/persist_parameter_client.cpp
)

target_link_libraries(client_default
  PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_include_directories(client_default
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

add_executable(client_with_node_options
  test/src/test_with_node_options.cpp
  test/src/persist_parameter_client.cpp
)

target_link_libraries(client_with_node_options
  PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
)

target_include_directories(client_with_node_options
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

install(TARGETS client_default client_with_node_options DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  test/launch
  DESTINATION share/${PROJECT_NAME}/
)
endif()

ament_package()
