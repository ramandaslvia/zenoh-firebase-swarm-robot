import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg = get_package_share_directory('swarm_robot')

    urdf_file = os.path.join(pkg, 'urdf', 'robot.urdf')
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen'
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen'
    )

    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-file', os.path.expanduser(
                        '~/robot_ws/src/swarm_robot/urdf/robot.urdf'),
                    '-name', 'swarm_robot',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.15'
                ],
                output='screen'
            )
        ]
    )

    bridge_cmdvel = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_cmdvel',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'
        ],
        output='screen'
    )

    bridge_imu = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_imu',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU'
        ],
        output='screen'
    )

    bridge_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_camera',
        arguments=[
            '/overhead_camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )

    bridge_odom_tf = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge_odom_tf',
        arguments=[
            '/model/swarm_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/model/swarm_robot/tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V'
        ],
        output='screen'
    )

    static_tf_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_world',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'world',
            '--child-frame-id', 'odom'
        ],
        output='screen'
    )

    aruco_detector = Node(
        package='swarm_robot',
        executable='aruco_detector',
        name='aruco_detector',
        output='screen'
    )

    # dijkstra_node: pendekatan lama berbasis grid Dijkstra + waypoint.
    # Dinonaktifkan karena arena tidak punya rintangan, sehingga
    # target_follower (kejar-lurus-ke-target) jauh lebih sederhana & stabil.
    dijkstra_node = Node(
        package='swarm_robot',
        executable='dijkstra_node',
        name='dijkstra_node',
        output='screen'
    )

    # target_follower: node AKTIF yang dipakai sekarang. Strategi dua-tahap
    # (putar di tempat sampai lurus, baru maju) — lihat README & laporan
    # praktikum untuk detail lengkap proses debugging-nya.
    target_follower = Node(
        package='swarm_robot',
        executable='target_follower',
        name='target_follower',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher,
        static_tf_world,
        spawn_robot,
        bridge_cmdvel,
        bridge_imu,
        bridge_camera,
        bridge_odom_tf,
        aruco_detector,
        target_follower,
    ])
