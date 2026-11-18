from launch import LaunchDescription
from launch_ros.actions import  Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution




def generate_launch_description():

    ld = LaunchDescription()
    


    ld.add_action(Node(
        package='dual_arm_control',
        executable='force_placement_node',
        output='screen',
        remappings=[('input_wrench', '/ekf/external_wrench_estimate'),
                    ('output_twist', '/desired_object_twist')]
    ))
   


    

    return ld