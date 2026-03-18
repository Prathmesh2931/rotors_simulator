cmake_minimum_required(VERSION 3.5)

project(rotors_swift_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rotors_swift_description REQUIRED)


# Install project configuration files
install(DIRECTORY
  config
  resource
  worlds
  src
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
  src/swarm_controller_keyboard.py
  src/generate_bridge_config.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
