cmake_minimum_required(VERSION 3.8)
project(cleannav_simulation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene_assets test/test_scene_assets.py)
endif()

install(
  DIRECTORY config launch models urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/dynamic_obstacle_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME dynamic_obstacle_controller
)

ament_package()
