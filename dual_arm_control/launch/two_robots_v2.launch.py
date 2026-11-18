from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch.conditions import IfCondition, UnlessCondition


from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import GroupAction
from launch_ros.actions import SetRemap
from launch_ros.actions import PushRosNamespace
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    
    robot1_namespace = 'robot1'
    robot2_namespace = 'robot2'
    default_rviz_config_path = PathJoinSubstitution([FindPackageShare('dual_arm_control'), 'rviz', 'rviz_two_robots.rviz'])

    robot1_sample_time = 0.005
    robot2_sample_time = 0.02
    
    
    robot1_frame_id = 'world'
    robot2_frame_id = 'base_link'



        
    ld = LaunchDescription()
    
    # Declare launch arguments
    
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                     description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)
    
    
    # launch iiwa nodes
    robot1_launch_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'single_robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_sample_time': str(robot1_sample_time),
                'urdf': 'urdf/iiwa_with_gripper.xacro',
                'flange_frame': 'iiwa_link8',
                'frame_id': robot1_frame_id,
                'joint_vel_limits': str([1.7104, 1.7104, 1.7453, 2.2689, 2.4435, 3.1416, 3.1416]),
            }.items()
            )
    robot1_launch_nodes = GroupAction(
            actions=[
                PushRosNamespace(robot1_namespace),
                robot1_launch_nodes,
        ])

    ld.add_action(robot1_launch_nodes)
    
    
    # launch yaskawa nodes
    robot2_launch_nodes = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'single_robot.launch.py'
                ])
            ]),
            launch_arguments={
                'robot_sample_time': str(robot2_sample_time),
                'urdf': 'urdf/sia5d.xacro',
                'flange_frame': 'tool0',
                'frame_id': robot2_frame_id,
                'joint_vel_limits': str([3.50, 3.50, 3.50, 3.50, 3.50, 4.015, 6.11]),
            }.items()
            )
    robot2_launch_nodes = GroupAction(
            actions=[
                PushRosNamespace(robot2_namespace),
                robot2_launch_nodes,
        ])

    ld.add_action(robot2_launch_nodes)
    
    # launch cartesian trajectory generator for object trajectory generation
    ld.add_action(Node(
        package='dual_arm_control',
        executable='trajectory_generator_node',
        name="cartesian_trajectory_object_control",
        parameters=[{'enable_joint_trajectory': False, 'trajectory.rate': 200.0}],
        remappings=[('cartesian_traj/pose', 'desired_object_pose'),
                    ('cartesian_traj/twist', 'desired_object_twist')],
        output='screen',
    ))
    
    # launch pose control node for robot1
    pose_force_controllers_robot1 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'launch_controllers_pose_force_single.launch.py'
                ])
            ]),
            launch_arguments={
                'pose_pose_control_gain_diag_vector': str([0.3, 0.3, 0.3, 0.20, 0.20, 0.20]),
                'force_force_control_gain_diag_vector': str([0.001, 0.001, 0.001, 0.02, 0.02, 0.02]),
            }.items()
    )
    pose_force_controllers_robot1 = GroupAction(
            actions=[
                PushRosNamespace(robot1_namespace),
                pose_force_controllers_robot1,
        ])
    ld.add_action(pose_force_controllers_robot1)
    
    # launch pose control node for robot2
    pose_force_controllers_robot2 = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('dual_arm_control'),
                    'launch',
                    'launch_controllers_pose_force_single.launch.py'
                ])
            ]),
            launch_arguments={
                'pose_pose_control_gain_diag_vector': str([0.3, 0.3, 0.3, 0.20, 0.20, 0.20]),
                'force_force_control_gain_diag_vector': str([0.0005, 0.0005, 0.0005, 0.01, 0.01, 0.01]),
            }.items()
    )
    pose_force_controllers_robot2 = GroupAction(
            actions=[
                PushRosNamespace(robot2_namespace),
                pose_force_controllers_robot2,
        ])
    ld.add_action(pose_force_controllers_robot2)





    # rviz
    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    ))

    # Transform between robots
    # ld.add_action(
    #     Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         arguments = ['1.63626', '-0.006', '0.013', '1.5282496', '-0.0356366', '0.0205613', robot1_frame_id, robot2_frame_id]
    #     )
    # )
    

    return ld
