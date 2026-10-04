cmake_minimum_required(VERSION 3.5)
project(cleannav_navigation)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(cleannav_interfaces REQUIRED)

install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  navigation_facade_node.py
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
    ackermann_reverse_config
    test/test_ackermann_reverse_config.py
  )
endif()

ament_package()
