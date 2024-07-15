import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # config = os.path.join(
    #     get_package_share_directory('my_hk_ros2'),
    #     'config',
    #     'my_camera.yaml'
    # )
    config = "/home/sqhl/Desktops/my_code/my_hk_ros2/src/my_hk_ros2/config/my_camera.yaml"
    
    return LaunchDescription([
        Node(
            package='my_hk_ros2',
            executable='hk_camera',
            parameters=[config]
        )
    ])

# import os
# from ament_index_python import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node
# from launch_ros.substitutions import YamlFile
# def generate_launch_description():

#   # Node to load parameters from YAML file
#     hk_camera_node = Node(
#         package='my_hk_ros2',
#         executable='hk_camera',
#         # name='hk_camera_node',
#         parameters=[YamlFile(os.path.join(get_package_share_directory('my_hk_ros2'), 'my_camera.yaml'))]
#     )

#     return LaunchDescription([
#         hk_camera_node
#     ])