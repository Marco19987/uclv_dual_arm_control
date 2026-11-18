from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.actions import LoadComposableNodes, Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare





def generate_launch_description():

    ld = LaunchDescription()

    ld.add_action(Node(
            package='dual_arm_control',
            executable='ros_simulator_ikfom',
            output='log',  # verbose per-cycle output, see ~/.ros/log
            parameters=[{
                'sample_time': 0.004,
                'publishing_sample_time': 0.01,
                'pose_measure_publishing_sample_time': 0.066666667,
                'robot_1_prefix': "robot1",
                'robot_2_prefix': "robot2",
                'sim_force_noise': False,
                'yaml_file_path': PathJoinSubstitution(
                    [FindPackageShare('dual_arm_control'), 'config', 'config_sim.yaml']),
                'object_name': "resin_block_1",
            }],
            # remappings for simulation
            # remappings=[('object_pose', 'ekf/object_pose'),
            #     ('internal_wrench_1_simulator','robot1/internal_wrench'),
            #     ('internal_wrench_2_simulator','robot2/internal_wrench'),
            #     ('e1To_topic','robot1/eeTo_topic'),
            #     ('e2To_topic','robot2/eeTo_topic'),]
        ))

    

    b2Tb1_tf_pub = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_b2Tb1',
        output='screen',
        parameters=[{'source_frame': 'base_link', 'target_frame' : 'world'}] ,
        remappings=[('pose_topic', 'b2Tb1_simulator')]
    )
    ld.add_action(b2Tb1_tf_pub)

    tf_bTo = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_bTo',
        output='screen',
        parameters=[{'source_frame': 'world', 'target_frame' : 'object_pose'}] ,
        remappings=[('pose_topic', 'object_pose')]
    )
    ld.add_action(tf_bTo)

    oTg1_tf_pub = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_bTo',
        output='screen',
        parameters=[{'source_frame': 'object_pose', 'target_frame' : 'grasp_1'}] ,
        remappings=[('pose_topic', 'oTg1_simulator')]
    )
    ld.add_action(oTg1_tf_pub)

    oTg2_tf_pub = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_bTo',
        output='screen',
        parameters=[{'source_frame': 'object_pose', 'target_frame' : 'grasp_2'}] ,
        remappings=[('pose_topic', 'oTg2_simulator')]
    )
    ld.add_action(oTg2_tf_pub)
    
    b1Te1_tf_pub = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_b1Te1',
        output='screen',
        parameters=[{'source_frame': 'world', 'target_frame' : 'end_effector_1'}] ,
        remappings=[('pose_topic', '/robot1/fkine_simulator')]
    )
    ld.add_action(b1Te1_tf_pub)
    
    b2Te2_tf_pub = Node(
        package='dual_arm_control',  # Replace with the actual package name
        executable='tf_publisher_from_topic',
        name='tf_b2Te2',
        output='screen',
        parameters=[{'source_frame': 'base_link', 'target_frame' : 'end_effector_2'}] ,
        remappings=[('pose_topic', '/robot2/fkine_simulator')]
    )
    ld.add_action(b2Te2_tf_pub)



    return ld