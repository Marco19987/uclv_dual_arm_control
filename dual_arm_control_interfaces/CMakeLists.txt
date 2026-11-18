cmake_minimum_required(VERSION 3.8)
project(dual_arm_control_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CartesianTrajectoryPoint.msg"
  "msg/CartesianTrajectory.msg"
  "srv/EKFService.srv"
  "srv/SetEndEffector.srv"
  "srv/ActivateJointController.srv"
  "action/JointTrajectory.action"
  "action/CartesianTrajectory.action"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs sensor_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
