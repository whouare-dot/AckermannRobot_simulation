import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Define Package Directories
    pkg_nav2_dir = get_package_share_directory('ackermann_navigation2')
    pkg_map_dir = get_package_share_directory('ackermann_mapping')
    pkg_robot_dir = get_package_share_directory('ackermann_robot')
    pkg_nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # 2. Configuration Variables
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Path to your nav2_params.yaml
    nav2_params_path = os.path.join(pkg_nav2_dir, 'params', 'param_top_akm_bs.yaml')

    # ================= NEW: 3D 转 2D 转换节点 =================
    # 这将订阅 /points_raw (3D) 并发布 /scan (2D) 给 Cartographer
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'laser_link', # 对应 sensors.xacro 中的 
            'transform_tolerance': 0.01,
            'min_height': -0.1,  # 根据你的机器人安装高度调整，保留地面以上的点
            'max_height': 1.0,   # 太高的障碍物忽略
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00349, # 分辨率
            'scan_time': 0.05,   # 20Hz (对应 xacro update_rate 20 )
            'range_min': 0.5,    # 对应 xacro range min [cite: 8]
            'range_max': 100.0,  # 对应 xacro range max [cite: 8]
            'use_inf': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', '/points_raw'), # 输入: sensors.xacro 发布的 3D 点云 
            ('scan', '/scan')            # 输出: Cartographer 需要的 2D 话题
        ]
    )
    # ==========================================================

    # 4. Include Cartographer (SLAM)
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_map_dir, 'launch', 'cartographer_2d.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 5. Include Nav2 (Navigation ONLY)
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params_path,
            'autostart': 'true'
        }.items()
    )

    # 6. Relay Node
    twist_stamper_node = Node(
        package='twist_stamper',
        executable='twist_stamper',
        name='twist_stamper',
        output='screen',
        remappings=[
            ('cmd_vel_in', '/cmd_vel'),
            ('cmd_vel_out', '/ackermann_steering_controller/reference')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 7. RViz
    rviz_config_path = os.path.join(pkg_nav2_bringup_dir, 'rviz', 'nav2_default_view.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 8.EKF 节点
    ekf_config_path = os.path.join(pkg_nav2_dir, 'config', 'ekf_config_rf2.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered')] 
    )

    return LaunchDescription([
        # 必须先启动转换节点，这样 Cartographer 启动时才能立即收到数据
        pointcloud_to_laserscan_node, 
        cartographer_launch,
        nav2_launch,
        twist_stamper_node,
        rviz_node,
        ekf_node
    ])