cmake_minimum_required(VERSION 3.8)
project(dual_arm_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(yaml-cpp REQUIRED)
find_package(ikfom_toolkit REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(dual_arm_control_interfaces REQUIRED)

include_directories(include ${YAML_CPP_INCLUDE_DIRS})

# Kinematic simulation of one robot arm (Pinocchio)
add_executable(robot_kinematics_node src/robot_kinematics_node.cpp)
target_link_libraries(robot_kinematics_node Eigen3::Eigen pinocchio::pinocchio)
ament_target_dependencies(robot_kinematics_node rclcpp geometry_msgs sensor_msgs std_srvs dual_arm_control_interfaces)

# Joint / Cartesian trajectory generator (actions)
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
target_link_libraries(trajectory_generator_node Eigen3::Eigen)
ament_target_dependencies(trajectory_generator_node rclcpp rclcpp_action geometry_msgs sensor_msgs
                          dual_arm_control_interfaces)

# Task sequencer (drives robots, simulator, EKF and controllers)
add_executable(demo_node src/demo_node.cpp)
target_link_libraries(demo_node Eigen3::Eigen yaml-cpp)
ament_target_dependencies(demo_node rclcpp rclcpp_action rcl_interfaces geometry_msgs sensor_msgs std_srvs ament_index_cpp
                          dual_arm_control_interfaces)

# Iterated error-state EKF on SE(3)
add_executable(ekf_server_ikfom src/ekf_server_ikfom.cpp)
target_link_libraries(ekf_server_ikfom Eigen3::Eigen yaml-cpp)
ament_target_dependencies(ekf_server_ikfom rclcpp geometry_msgs std_msgs std_srvs visualization_msgs
                          ikfom_toolkit dual_arm_control_interfaces)

# Robots - spring - object simulator
add_executable(ros_simulator_ikfom src/ros_simulator_ikfom.cpp)
target_link_libraries(ros_simulator_ikfom Eigen3::Eigen yaml-cpp)
ament_target_dependencies(ros_simulator_ikfom rclcpp geometry_msgs std_msgs std_srvs visualization_msgs
                          ament_index_cpp ikfom_toolkit dual_arm_control_interfaces)

# Controllers
add_executable(pose_control_node_single src/pose_control_node_single.cpp)
target_link_libraries(pose_control_node_single Eigen3::Eigen)
ament_target_dependencies(pose_control_node_single rclcpp geometry_msgs std_srvs ikfom_toolkit)

add_executable(internal_force_control_node_single src/internal_force_control_node_single.cpp)
target_link_libraries(internal_force_control_node_single Eigen3::Eigen)
ament_target_dependencies(internal_force_control_node_single rclcpp geometry_msgs std_srvs)

add_executable(sum_twist_node src/sum_twist_node.cpp)
ament_target_dependencies(sum_twist_node rclcpp geometry_msgs)

add_executable(force_placement_node src/force_placement_node.cpp)
target_link_libraries(force_placement_node Eigen3::Eigen)
ament_target_dependencies(force_placement_node rclcpp geometry_msgs std_srvs)

# Utilities
add_executable(tf_publisher_from_topic src/tf_publisher_from_topic.cpp)
target_link_libraries(tf_publisher_from_topic Eigen3::Eigen)
ament_target_dependencies(tf_publisher_from_topic rclcpp geometry_msgs tf2 tf2_ros)

install(
  TARGETS robot_kinematics_node trajectory_generator_node demo_node ekf_server_ikfom ros_simulator_ikfom
          pose_control_node_single internal_force_control_node_single sum_twist_node force_placement_node
          tf_publisher_from_topic
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf meshes rviz plotjuggler
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
