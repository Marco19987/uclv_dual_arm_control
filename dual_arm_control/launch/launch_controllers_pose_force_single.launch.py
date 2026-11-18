from launch import LaunchDescription
from launch_ros.actions import  Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution




configurable_object_pose_control_node_parameters = [
    {'name': 'pose_control_gain_diag_vector', 'default': [0.5, 0.5, 0.5, 0.25, 0.25, 0.25], 'description': 'elements of the control gain diagonal matrix'},
    {'name': 'linear_vel_saturation', 'default': 0.1, 'description': 'linear velocity saturation'},
    {'name': 'angular_vel_saturation', 'default': 1.0, 'description': 'angular velocity saturation'},
]
#[0.5, 0.5, 0.5, 0.25, 0.25, 0.25]

configurable_internal_force_control_node_parameters = [
    {'name': 'force_control_gain_diag_vector', 'default': [7.0E-4,7.0E-4,7.0E-4,0.02,0.02,0.02], 'description': 'elements of the control gain diagonal matrix'},
    {'name': 'linear_vel_saturation', 'default': 0.005, 'description': 'linear velocity saturation'},
    {'name': 'angular_vel_saturation', 'default': 0.05, 'description': 'angular velocity saturation'},
]


def convert_parameters(parameters):
    return {param['name']: param['default'] for param in parameters}

def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

def set_configurable_parameters(parameters):
    return dict([(param['name'], LaunchConfiguration(param['name'])) for param in parameters])

def generate_launch_description():

    ld = LaunchDescription()
    
    # Declare all configurable parameters for both nodes
    for param in configurable_object_pose_control_node_parameters:
        ld.add_action(DeclareLaunchArgument("pose_"+param['name'], default_value=str(param['default']), description=param['description']))
    for param in  configurable_internal_force_control_node_parameters:
        ld.add_action(DeclareLaunchArgument("force_"+param['name'], default_value=str(param['default']), description=param['description']))

    # Collect parameters from launch configuration for each node
    object_pose_params = [{param['name']: LaunchConfiguration("pose_"+param['name'])} for param in configurable_object_pose_control_node_parameters]
    internal_force_params = [{param['name']: LaunchConfiguration("force_"+param['name'])} for param in configurable_internal_force_control_node_parameters]

    ld.add_action(Node(
        package='dual_arm_control',
        executable='pose_control_node_single',
        output='screen',
        parameters=object_pose_params,
        remappings=[('object_pose', '/ekf/object_pose'),
                    ('desired_object_pose', '/desired_object_pose'),
                    ('eeTo_topic', 'eeTo_topic'),
                    ('desired_twist', '/desired_object_twist'),
                    ('fkine_twist_body_frame','twist_body_pose')]
    ))
    ld.add_action(Node(
        package='dual_arm_control',
        executable='internal_force_control_node_single',
        output='screen',
        parameters=internal_force_params,
        remappings=[('fkine_twist_body_frame','twist_body_force')]
        
    ))
    
    ld.add_action(Node(
        package='dual_arm_control',
        executable='sum_twist_node',
        output='screen',
        remappings=[('twist1','twist_body_pose'),
                    ('twist2','twist_body_force'),
                    ('twist_sum','twist_body')]
        
    ))


    

    return ld