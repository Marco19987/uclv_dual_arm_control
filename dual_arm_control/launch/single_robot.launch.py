"""Simulated robot arm: robot_state_publisher, kinematic simulation (Pinocchio), trajectory generator and
end-effector pose controller. Meant to be included inside a namespace (robot1 / robot2)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

configurable_robot_pose_control_parameters = [
    {'name': 'pose_control_gain_diag_vector', 'default': [0.5, 0.5, 0.5, 0.25, 0.25, 0.25],
     'description': 'elements of the control gain diagonal matrix'},
    {'name': 'linear_vel_saturation', 'default': 0.1, 'description': 'linear velocity saturation'},
    {'name': 'angular_vel_saturation', 'default': 1.0, 'description': 'angular velocity saturation'},
    {'name': 'eeTo_is_identity', 'default': True, 'description': 'eeTo_is_identity'},
]


def generate_launch_description():
    ld = LaunchDescription()

    for param in configurable_robot_pose_control_parameters:
        ld.add_action(DeclareLaunchArgument('robot_' + param['name'], default_value=str(param['default']),
                                            description=param['description']))
    robot_control_params = [{param['name']: LaunchConfiguration('robot_' + param['name'])}
                            for param in configurable_robot_pose_control_parameters]

    ld.add_action(DeclareLaunchArgument('robot_sample_time', default_value='0.005',
                                        description='Sample time of the robot [s]'))
    ld.add_action(DeclareLaunchArgument('urdf', default_value='urdf/iiwa_with_gripper.xacro',
                                        description='Robot xacro, relative to the dual_arm_control package'))
    ld.add_action(DeclareLaunchArgument('flange_frame', default_value='iiwa_link8',
                                        description='Flange frame of the robot in the URDF'))
    ld.add_action(DeclareLaunchArgument('frame_id', default_value='world',
                                        description='Base frame of the robot (frame_id of the published poses)'))
    ld.add_action(DeclareLaunchArgument('joint_vel_limits', default_value='[1.7104, 1.7104, 1.7453, 2.2689, 2.4435, 3.1416, 3.1416]',
                                        description='Joint velocity limits [rad/s]'))

    robot_sample_time = LaunchConfiguration('robot_sample_time')
    robot_rate = PythonExpression(['1.0 / ', robot_sample_time])
    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution([FindPackageShare('dual_arm_control'), LaunchConfiguration('urdf')])]),
        value_type=str)

    # TF of the robot links (for RViz)
    ld.add_action(Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
    ))

    # kinematic simulation of the arm: joint integrator, inverse/forward kinematics, null-space joint controller
    ld.add_action(Node(
        package='dual_arm_control',
        executable='robot_kinematics_node',
        name='robot_kinematics',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'flange_frame': LaunchConfiguration('flange_frame'),
            'frame_id': LaunchConfiguration('frame_id'),
            'sample_time': ParameterValue(robot_sample_time, value_type=float),
            'joint_vel_limits': ParameterValue(LaunchConfiguration('joint_vel_limits'), value_type=None),
            'null_space_gain': 0.3,
            'null_space_saturation': 0.17,
        }],
    ))

    # joint and Cartesian trajectories of the arm
    ld.add_action(Node(
        package='dual_arm_control',
        executable='trajectory_generator_node',
        name='trajectory_generator',
        output='screen',
        parameters=[{'trajectory.rate': ParameterValue(robot_rate, value_type=float)}],
    ))

    # end-effector pose controller (tracks cartesian_traj/pose when activated by the task sequencer)
    ld.add_action(Node(
        package='dual_arm_control',
        executable='pose_control_node_single',
        output='screen',
        name='robot_pose_control',
        parameters=robot_control_params,
        remappings=[('object_pose', 'fkine'),
                    ('desired_object_pose', 'cartesian_traj/pose'),
                    ('desired_twist', 'cartesian_traj/twist'),
                    ('fkine_twist_body_frame', 'twist_body')],
    ))

    return ld
