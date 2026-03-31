# keyboard_control.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    # 键盘控制节点
    keyboard_control = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        parameters=[{
            'stamped': True,           # 启用时间戳，发送 TwistStamped 消息 
            'frame_id': 'base_link'    # 指定消息头中的 frame_id
        }],
        remappings=[
            # 将默认的 /cmd_vel 重映射到阿克曼控制器的输入话题
            ('/cmd_vel', '/ackermann_steering_controller/reference') 
        ],
        output='screen',
        prefix='xterm -e',  # 在新终端窗口中运行
    )
    
    ld = LaunchDescription()
    ld.add_action(keyboard_control)
    
    return ld