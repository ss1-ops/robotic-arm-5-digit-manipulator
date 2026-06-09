cmake_minimum_required(VERSION 3.8)
project(moveo_description)

find_package(ament_cmake REQUIRED)

# Install URDF and meshes
install(DIRECTORY urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

# Install ros2_control config
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
