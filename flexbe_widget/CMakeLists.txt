cmake_minimum_required(VERSION 3.5)
project(flexbe_widget)
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

INSTALL(PROGRAMS
    bin/be_action_server
    bin/be_compress
    bin/be_launcher
    bin/breakpoint
    bin/create_repo
    bin/evaluate_logs
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
#install(PROGRAMS
#  scripts/py_node.py
#  DESTINATION lib/${PROJECT_NAME}
#)

ament_package()
