cmake_minimum_required(VERSION 3.8)
project(augdd_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# maps/ gets populated by map_saver_cli at runtime; make sure the dir exists
install(DIRECTORY DESTINATION share/${PROJECT_NAME}/maps)

ament_package()
