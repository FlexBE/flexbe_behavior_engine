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
    bin/flexbe_status_listener
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  set(ament_cmake_flake8_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.flake8")
  ament_flake8()
  ament_pep257()
  ament_copyright()
endif()

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

ament_package()
