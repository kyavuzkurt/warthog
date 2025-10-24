cmake_minimum_required(VERSION 3.10)
project(warthog_msgs)

# Suppress Python CMake policy warnings
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Lights.msg"
  "msg/RGB.msg"
  "msg/Status.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()
