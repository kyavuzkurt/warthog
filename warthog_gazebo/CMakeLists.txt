cmake_minimum_required(VERSION 3.10)
project(warthog_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and worlds
install(DIRECTORY 
  launch 
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/odom_to_tf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
