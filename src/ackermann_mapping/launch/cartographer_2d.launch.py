import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 替换为你的功能包名
    package_name = 'ackermann_mapping' 
    
    # 配置文件路径
    cartographer_config_dir = os.path.join(get_package_share_directory(package_name), 'config')
    configuration_basename = 'ackermann_2d.lua'

    return LaunchDescription([
        
        # 1. Cartographer Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': True}], # 仿真必须设为 True
            arguments=[
                '-configuration_directory', cartographer_config_dir,
                '-configuration_basename', configuration_basename
            ],
            remappings=[
                ('scan', '/scan'),       # 对应 sensors.xacro 
                ('odom', '/odometry/filtered'), # 对应 EKF 输出的话题 (通常 robot_localization 输出这个)
                ('imu', '/imu/data'),    # 对应 sensors.xacro 
            ]
        ),

        # 2. Occupancy Grid Node (生成栅格地图)
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[
                {'use_sim_time': True},
                {'resolution': 0.05}
            ]
        ),
    ])