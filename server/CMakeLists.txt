cmake_minimum_required(VERSION 3.5)
project(parameter_server VERSION 1.0.0)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils)
find_package(std_msgs REQUIRED)
find_package(rmw REQUIRED)

find_package(Boost REQUIRED COMPONENTS program_options filesystem)
find_package(yaml_cpp_vendor REQUIRED)

include_directories(include)
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${yaml_cpp_vendor_INCLUDE_DIRS})

add_executable(server
 src/parameter_server.cpp
 src/main.cpp
)
ament_target_dependencies(server rclcpp rclcpp_components std_msgs)
target_link_libraries(server ${Boost_LIBRARIES} ${yaml_cpp_vendor_LIBRARIES})

install(TARGETS server DESTINATION lib/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  param
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
