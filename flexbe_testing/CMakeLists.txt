cmake_minimum_required(VERSION 3.5)
project(flexbe_testing)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

INSTALL(PROGRAMS
    bin/testing_node
    DESTINATION lib/${PROJECT_NAME}
)

INSTALL(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

INSTALL(DIRECTORY
    test
    DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
