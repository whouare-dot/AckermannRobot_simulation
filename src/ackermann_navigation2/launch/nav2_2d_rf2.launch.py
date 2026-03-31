import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ackermann_navigation2_dir = get_package_share_directory('ackermann_navigation2')
    ackermann_robot_dir = get_package_share_directory('ackermann_robot')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true') 
    map_yaml_path = LaunchConfiguration('map',default=os.path.join(ackermann_robot_dir,'maps','mini_world.yaml'))
    nav2_param_path = LaunchConfiguration('params_file',default=os.path.join(ackermann_navigation2_dir,'params','param_top_akm_bs.yaml'))
    rviz_config_dir = os.path.join(nav2_bringup_dir,'rviz','nav2_default_view.rviz')

    nav2_bringup_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_bringup_dir,'/launch','/bringup_launch.py']),
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path
            }.items(),
    )

    # 重要：通过这个节点重映射 AMCL 的话题订阅
    # 注意：AMCL 默认监听 /initialpose。我们要让它监听 ICP 优化后的结果。
    # 这里通过节点重命名或参数服务器重映射实现

    # 重新定义的 icp_relocalizer
    icp_relocalizer_node = Node(
        package='ackermann_navigation2',
        executable='icp_relocalizer',
        name='icp_relocalizer',
        output='screen',
        parameters=[{
            #所有匹配点对之间距离的均方误差 (Mean Squared Error)。得分越低，代表雷达点云与地图重合得越完美。
            #如果得分超过这个阈值，节点会判定本次配准“不可靠”，从而拒绝发布位姿，
            'max_fitness_score': 0.12,  #最大适应度得分（质量阈值）
            #为扫描中的每一个点寻找地图上距离最近的点。如果这两个点之间的距离大于 max_correspondence_dist，算法就会认为这两个点不属于同一个物体，从而忽略这对点
            #调大会提高算法对粗略初值的容忍度
            'max_correspondence_dist': 2.5,  #最大对应点距离（搜索半径）
            'use_sim_time': True
    }]
    )

    # 将 AMCL 的输入话题进行重映射
    # 在 generate_launch_description 返回前增加这个逻辑：通过 GroupAction 包装 nav2_bringup_launch 进行重映射
    from launch.actions import GroupAction
    from launch_ros.actions import SetRemap
    
    nav2_with_remapping = GroupAction(
        actions=[
            SetRemap(src='/initialpose', dst='/initialpose_refined'),
            nav2_bringup_launch
        ]
    )
    
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

    rviz_node =  Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen')
    
    # ================= EKF 节点 =================
    ekf_config_path = os.path.join(ackermann_navigation2_dir, 'config', 'ekf_config_rf2.yaml')
    
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config_path, {'use_sim_time': True}],
        remappings=[('/odometry/filtered', '/odometry/filtered')] 
    )
    
    #将 3D 点云转换为 2D Scan 给 AMCL 使用
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[{
            'target_frame': 'laser_link',
            'transform_tolerance': 0.2,
            'min_height': -0.1,  # 只有在这个高度范围内的点会被压缩成 2D 扫描
            'max_height': 1.0,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00349, # 约 0.2度分辨率
            'scan_time': 0.05,
            'range_min': 0.5,
            'range_max': 100.0,
            'use_inf': True,
            'use_sim_time': use_sim_time
        }],
        remappings=[
            ('cloud_in', '/points_raw'), # 输入 3D 点云
            ('scan', '/scan')            # 输出 2D 扫描给 AMCL
        ]
    )

    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        arguments=['--ros-args', '--log-level', 'error'],
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False, # 注意：必须为 False，因为 TF 由 EKF 发布
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 20.0
        }]
    )
    
    return LaunchDescription([
        nav2_with_remapping,
        icp_relocalizer_node,
        rviz_node,
        twist_stamper_node,
        ekf_node,
        pointcloud_to_laserscan_node,
        rf2o_node
    ])