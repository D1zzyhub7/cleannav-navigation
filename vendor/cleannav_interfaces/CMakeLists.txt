cmake_minimum_required(VERSION 3.8)
project(cleannav_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateWithinPolygon.action"
  "msg/CleaningTarget.msg"
  "msg/CleaningTargetArray.msg"
  "msg/NavigationBoundary.msg"
  "msg/NavigationBoundaryState.msg"
  "msg/PerceptionHealth.msg"
  "msg/RobotStatus.msg"
  "msg/SafetyStatus.msg"
  "msg/TaskCommand.msg"
  "msg/TaskSequenceStatus.msg"
  "msg/TaskSequenceStep.msg"
  "msg/TaskStatus.msg"
  "srv/SafetyLease.srv"
  "srv/SubmitNavigationGoal.srv"
  "srv/SubmitTaskSequence.srv"
  DEPENDENCIES builtin_interfaces std_msgs action_msgs geometry_msgs
)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
