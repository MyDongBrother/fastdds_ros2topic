from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # 创建LaunchDescription对象，并将节点配置添加到其中
    return LaunchDescription([
        Node(
            #功能包名
            package='fastdds_demo',
            #节点名必须与cmake文件中add_executable生成的可执行程序名称一致
            executable='fastdds_demo',
            #节点的别名，用于在命令行工具和rqt中显示节点的名称
            name='fastdds_demo'
        )
    ])
