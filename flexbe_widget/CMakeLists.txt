cmake_minimum_required(VERSION 3.10.2)
project(flexbe_widget)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(PROGRAMS
    bin/be_action_server
    bin/be_compress
    bin/be_launcher
    bin/breakpoint
    bin/create_repo
    bin/evaluate_logs
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
