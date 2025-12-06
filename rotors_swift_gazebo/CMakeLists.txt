cmake_minimum_required(VERSION 3.5)

project(rotors_swift_gazebo)

find_package(ament_cmake REQUIRED)
find_package(rotors_swift_description REQUIRED)

# Install project launch files
install(
  DIRECTORY
    launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Install project configuration files
install(
  DIRECTORY
    config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY
    resource/
  DESTINATION share/${PROJECT_NAME}/resource
)

install(
  DIRECTORY worlds/
  DESTINATION share/${PROJECT_NAME}/worlds
)

install(
  DIRECTORY src/
  DESTINATION share/${PROJECT_NAME}/src
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  PROGRAMS
  src/swarm_controller_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
