cmake_minimum_required(VERSION 3.8)
project(dmg_mori_5axis)

find_package(ament_cmake REQUIRED)

# Installer tous les dossiers dans le share
install(DIRECTORY
  urdf
  meshes
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
