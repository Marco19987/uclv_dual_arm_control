"""Run the whole cooperative transport simulation with a single command.

    ros2 launch dual_arm_control simulation.launch.py

It starts, in order:
  1. the two simulated robots (iiwa + SIA5F), their controllers, the object trajectory generator and RViz
  2. the robots-spring-object simulator
  3. the IESEKF on SE(3)
  4. the force-guarded placement node
  5. the task sequencer (demo_node), which executes config/task_resin_block_3.yaml

Launch arguments:
  internal_force_control:=true|false   enable the internal wrench controllers (default true)
  object_pose_control:=true|false      enable the object pose controllers (default true)
  auto_continue:=true|false            run the task without waiting for ENTER (default true)
  auto_continue_delay:=<s>             pause between task steps in auto mode (default 2.0)
  start_demo:=true|false               start demo_node (default true). Set to false and run
                                       `ros2 run dual_arm_control demo_node true true true true false false true`
                                       in another terminal to step through the task with ENTER.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def include(launch_file):
    return IncludeLaunchDescription(PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare('dual_arm_control'), 'launch', launch_file])))


def generate_launch_description():
    args = [
        DeclareLaunchArgument('internal_force_control', default_value='true',
                              description='Enable the internal wrench controllers'),
        DeclareLaunchArgument('object_pose_control', default_value='true',
                              description='Enable the object pose controllers'),
        DeclareLaunchArgument('auto_continue', default_value='true',
                              description='Execute the task without waiting for ENTER'),
        DeclareLaunchArgument('auto_continue_delay', default_value='2.0',
                              description='Pause [s] between task steps in auto mode'),
        DeclareLaunchArgument('start_demo', default_value='true',
                              description='Start the task sequencer (demo_node)'),
    ]

    demo_node = Node(
        package='dual_arm_control',
        executable='demo_node',
        name='cooperative_robots_demo',
        output='screen',
        emulate_tty=True,
        condition=IfCondition(LaunchConfiguration('start_demo')),
        # <use_internal_force_control> <use_object_pose_control> <use_ekf> <use_estimated_b1Tb2>
        # <use_pivoting> <use_aruco> <simulation>
        arguments=[LaunchConfiguration('internal_force_control'), LaunchConfiguration('object_pose_control'),
                   'true', 'true', 'false', 'false', 'true'],
        parameters=[{
            'auto_continue': ParameterValue(LaunchConfiguration('auto_continue'), value_type=bool),
            'auto_continue_delay': ParameterValue(LaunchConfiguration('auto_continue_delay'), value_type=float),
        }],
    )

    return LaunchDescription(args + [
        include('two_robots_v2.launch.py'),
        # give the robot containers time to come up before the simulator starts integrating
        TimerAction(period=5.0, actions=[include('launch_ros_simulator.launch.py')]),
        TimerAction(period=7.0, actions=[include('bringup_ekf_server.launch.py'),
                                         include('launch_force_placement_node.launch.py')]),
        # demo_node waits for all the services it needs before starting the task
        TimerAction(period=10.0, actions=[demo_node]),
    ])
