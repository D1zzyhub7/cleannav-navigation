cmake_minimum_required(VERSION 3.5)
project(cleannav_navigation)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_mppi_controller REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cleannav_interfaces REQUIRED)
add_library(cleannav_navigation_boundary_layer SHARED src/navigation_boundary_layer.cpp)
target_compile_features(cleannav_navigation_boundary_layer PUBLIC cxx_std_17)
ament_target_dependencies(cleannav_navigation_boundary_layer nav2_costmap_2d tf2_geometry_msgs cleannav_interfaces pluginlib)
pluginlib_export_plugin_description_file(nav2_costmap_2d navigation_boundary_plugin.xml)
install(TARGETS cleannav_navigation_boundary_layer LIBRARY DESTINATION lib)
add_library(cleannav_bounded_mppi_controller SHARED src/bounded_mppi_controller.cpp)
target_compile_features(cleannav_bounded_mppi_controller PUBLIC cxx_std_17)
target_include_directories(cleannav_bounded_mppi_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cleannav_bounded_mppi_controller nav2_mppi_controller nav2_core pluginlib)
target_compile_options(cleannav_bounded_mppi_controller PRIVATE -fconcepts)
pluginlib_export_plugin_description_file(nav2_core bounded_mppi_plugin.xml)
install(TARGETS cleannav_bounded_mppi_controller LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_export_include_directories(include)


install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  navigation_facade_node.py
  polygon_constraint.py
  boundary_runtime.py
  patrol_path_segments.py
  demo_hil_navigation_client.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    navigation_facade_unit
    test/test_navigation_facade.py
  )
  ament_add_pytest_test(
    navigation_facade_costmap_contract
    test/test_navigation_facade_costmap_contract.py
  )
  ament_add_pytest_test(
    ackermann_reverse_config
    test/test_ackermann_reverse_config.py
  )
  ament_add_pytest_test(execution_mode test/test_execution_mode.py)
  ament_add_pytest_test(patrol_path_segments ${CMAKE_CURRENT_SOURCE_DIR}/../tests/patrol)
  ament_add_pytest_test(facade_follow_lifecycle test/test_facade_follow_lifecycle.py)
endif()

ament_package()
