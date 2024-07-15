from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path

def generate_launch_description():
        
    return LaunchDescription([
        Node(
            package='my_hk_ros2',
            executable='hk_camera',
            #parameters记录了配置文件yaml的路径
            parameters=[str(get_package_share_path('my_hk_ros2') / 'config/my_camera.yaml')
            ])
    ])