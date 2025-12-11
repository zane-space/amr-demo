import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'amr_demo'
    pkg_share = get_package_share_directory(pkg_name)

    # 處理 URDF 檔案
    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    # 處理 World 檔案路徑
    world_file = os.path.join(pkg_share, 'worlds', 'obstacle_world.world')

    # 0. 
    env = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1'
    )

    # 1. 啟動 Robot State Publisher (發布 TF)
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    # 2. 啟動 Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': ['-r --render-engine ogre ', world_file]
        }.items()
    )

    # 3. 在 Gazebo 中生成機器人
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'amr_demo'],
        output='screen'
    )

    # 4. 啟動自定義避障節點 (C++)
    avoidance_node = Node(
        package='amr_demo',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 5. 啟動 ROS - Gazebo 橋接器
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 橋接 cmd_vel (ROS -> Gazebo)
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # 橋接 scan (Gazebo -> ROS)
            '/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            # 橋接 odom (Gazebo -> ROS) (選用)
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        env,
        rsp,
        gazebo,
        spawn_entity,
        avoidance_node,
        bridge
    ])

