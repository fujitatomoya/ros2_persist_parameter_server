cmake_minimum_required(VERSION 3.5)
project(parameter_server VERSION 1.0.1)

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

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(yaml_cpp_vendor REQUIRED)

add_executable(server
 src/parameter_server.cpp
 src/main.cpp
)

# yaml-cpp updates CMake thing significantly on v0.8.0 or later.
# so we ended up having the if statement to process differently instead of creating branches.
# see https://github.com/jbeder/yaml-cpp/releases/tag/0.8.0
find_package(yaml-cpp 0.8.0 QUIET)
if (yaml-cpp_FOUND)
  message(STATUS "yaml-cpp package is greater equal than version 0.8.0")
  target_link_libraries(server
    rclcpp::rclcpp
    rcutils::rcutils
    ${Boost_LIBRARIES}
    yaml-cpp::yaml-cpp
  )
else()
  message(STATUS "yaml-cpp package is less than version 0.8.0")
  find_package(yaml-cpp REQUIRED)
  target_link_libraries(server
    rclcpp::rclcpp
    rcutils::rcutils
    ${Boost_LIBRARIES}
    yaml-cpp
  )
endif()

target_include_directories(server
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

ament_target_dependencies(server rclcpp rclcpp_components std_msgs rcutils)

install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
