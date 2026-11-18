"""Launch the IESEKF on SE(3) (ekf_server_ikfom) wired to the robots-object simulator."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dual_arm_control',
            executable='ekf_server_ikfom',
            name='ekf_server',
            output='log',  # verbose per-cycle output, see ~/.ros/log
            parameters=[
                {'sample_time': 0.004},              # filter sample time [s]
                {'stop_calibration_param': 0.003},
                {'occlusion_exp_limit': 15},
                {'occlusion_base': 1.584893192},
                {'publishing_sample_time': 0.01},
                {'robot_1_prefix': '/robot1'},
                {'robot_2_prefix': '/robot2'},
                {'base_frame_name': 'world'},
                {'publish_eeTo': True},
                # State noise covariance (position, orientation) for each SE(3)/twist block
                {'covariance_state_diagonal': [1E-8, 1E-7,   # bTo   (object pose)
                                               1E-6, 1E-5,   # ovo   (object body twist)
                                               1E-8, 1E-7,   # b1Te1 (robot 1 end effector)
                                               1E-8, 1E-7,   # b2Te2 (robot 2 end effector)
                                               1E-7, 1E-6,   # b2Tb1 (relative pose of the robots)
                                               1E-9, 1E-8,   # oTg1, oTg2 (grasp frames)
                                               ]},
                # Measurement noise covariance (16 values)
                {'covariance_measure_diagonal': [2E-9, 2E-9, 2E-9, 5E-8, 5E-8, 5E-8,          # visual object pose (per marker)
                                                 0.0025, 0.0025, 0.000225, 1E-6, 1E-6, 9E-8,  # tactile wrench (fx fy fz tx ty tz)
                                                 1E-10, 1E-9,                                 # robot 1 fkine (pos, orient)
                                                 1E-10, 1E-9]},                               # robot 2 fkine (pos, orient)
            ],
            remappings=[
                # EKF outputs used by the internal wrench / pose controllers
                ('/ekf/internal_wrench_estimate_1', 'robot1/internal_wrench'),
                ('/ekf/internal_wrench_estimate_2', 'robot2/internal_wrench'),
                ('/ekf/e1To_topic', 'robot1/eeTo_topic'),
                ('/ekf/e2To_topic', 'robot2/eeTo_topic'),
                # Inputs from the robots and from the robots-object simulator
                ('/robot1/twist_fkine', '/robot1/command/fkine_twist_tactile_sensor_body_frame'),
                ('/robot2/twist_fkine', '/robot2/command/fkine_twist_tactile_sensor_body_frame'),
                ('/robot1/fkine', '/robot1/fkine_simulator'),
                ('/robot2/fkine', '/robot2/fkine_simulator'),
            ]
        )
    ])
