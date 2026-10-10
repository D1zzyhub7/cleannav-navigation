cmake_minimum_required(VERSION 3.8)
project(cleannav_simulation)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scene_assets test/test_scene_assets.py)
  ament_add_pytest_test(test_full_sim test/test_full_sim.py)
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

install(
  PROGRAMS scripts/showcase_person_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME showcase_person_controller
)

install(
  PROGRAMS scripts/bicycle_patrol_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME bicycle_patrol_controller
)

install(
  PROGRAMS scripts/leaf_cleanup_demo_controller.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME leaf_cleanup_demo_controller
)

install(PROGRAMS scripts/simulation_status_monitor.py
  DESTINATION lib/${PROJECT_NAME} RENAME simulation_status_monitor)
install(PROGRAMS scripts/simulation_chassis_test.py
  DESTINATION lib/${PROJECT_NAME} RENAME simulation_chassis_test)

ament_package()
