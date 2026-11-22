cmake_minimum_required(VERSION 3.20)
project(persist_parameter_server)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

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
find_package(rcl_interfaces REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(yaml_cpp_vendor REQUIRED)

# yaml-cpp updates CMake thing significantly on v0.8.0 or later.
# so we ended up having the if statement to process differently instead of creating branches.
# see https://github.com/jbeder/yaml-cpp/releases/tag/0.8.0
find_package(yaml-cpp 0.8.0 QUIET)
if(yaml-cpp_FOUND)
  message(STATUS "yaml-cpp package is greater equal than version 0.8.0")
  set(YAML_CPP_LIBRARIES yaml-cpp::yaml-cpp)
else()
  message(STATUS "yaml-cpp package is less than version 0.8.0")
  find_package(yaml-cpp REQUIRED)
  set(YAML_CPP_LIBRARIES yaml-cpp)
endif()

# Parameter server core library, shared by the server executable and the tests.
add_library(${PROJECT_NAME}_core STATIC
  server/src/parameter_server.cpp
)

target_include_directories(${PROJECT_NAME}_core
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/server/include>
)

target_link_libraries(${PROJECT_NAME}_core
  PUBLIC
  rclcpp::rclcpp
  rclcpp_components::component
  rcutils::rcutils
  ${YAML_CPP_LIBRARIES}
  ${std_msgs_TARGETS}
  ${std_srvs_TARGETS}
  Boost::program_options
  Boost::filesystem
)

add_executable(server
  server/src/main.cpp
)

target_link_libraries(server
  PRIVATE
  ${PROJECT_NAME}_core
)

install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  server/launch
  server/param
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_ros REQUIRED)
  # Unit and system tests run the parameter server in-process against a parameter client,
  # each test runs with a dedicated ROS_DOMAIN_ID so it does not interfere with running nodes.
  ament_add_ros_isolated_gtest(test_parameter_server
    test/test_parameter_server.cpp
    test/persist_parameter_client.cpp
    TIMEOUT 180
  )
  if(TARGET test_parameter_server)
    target_link_libraries(test_parameter_server ${PROJECT_NAME}_core)
  endif()
endif()

ament_package()
