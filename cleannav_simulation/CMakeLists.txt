cmake_minimum_required(VERSION 3.8)
project(cleannav_simulation)

find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(cleannav_laser_self_filter SHARED src/laser_self_filter.cpp)
target_compile_features(cleannav_laser_self_filter PUBLIC cxx_std_17)
target_include_directories(cleannav_laser_self_filter PRIVATE include ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(cleannav_laser_self_filter ${GAZEBO_LIBRARIES})
ament_target_dependencies(cleannav_laser_self_filter gazebo_ros rclcpp sensor_msgs std_msgs)
install(TARGETS cleannav_laser_self_filter LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_laser_self_filter test/test_laser_self_filter.cpp)
  target_include_directories(test_laser_self_filter PRIVATE include)
  ament_add_pytest_test(test_scene_assets test/test_scene_assets.py)
  ament_add_pytest_test(test_full_sim test/test_full_sim.py)
  ament_add_pytest_test(test_rviz_goal_adapter test/test_rviz_goal_adapter.py)
  ament_add_pytest_test(test_motion_diagnostics test/test_motion_diagnostics.py)
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
install(PROGRAMS scripts/simulation_task_client.py
  DESTINATION lib/${PROJECT_NAME} RENAME simulation_task_client)
install(PROGRAMS scripts/simulation_motion_diagnostics.py
  DESTINATION lib/${PROJECT_NAME} RENAME simulation_motion_diagnostics)
install(PROGRAMS scripts/simulation_chassis_test.py
  DESTINATION lib/${PROJECT_NAME} RENAME simulation_chassis_test)

ament_package()

install(PROGRAMS scripts/rviz_goal_adapter.py
  DESTINATION lib/${PROJECT_NAME} RENAME rviz_goal_adapter)
