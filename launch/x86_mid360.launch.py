from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition  # 必须添加此导入
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 参数声明
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='是否启动RViz'
    )

    
    # 获取配置文件路径
    mapping_config = os.path.join(
        get_package_share_directory('lidar_slam'),
        'config',
        'param_slam_x86_mid360_ros2.yaml'
    )
    
    # camera_config = os.path.join(
    #     get_package_share_directory('fast_livo'),
    #     'config',
    #     'camera_pinhole.yaml'
    # )
    
    # rviz_config = os.path.join(
    #     get_package_share_directory('fast_livo'),
    #     'rviz_cfg',
    #     'fast_livo2.rviz'
    # )

    
    # 节点定义
    lidar_slam_node = Node(
        package='lidar_slam',
        executable='lidar_slam_node',
        name='localization_module',
        output='screen',
        # prefix="gdb -ex run --args",
        
        # parameters=[avia_config, camera_config],
        parameters=[mapping_config]
    )
    
    
    # republish_node = Node(
    #     package='image_transport',
    #     executable='republish',
    #     name='republish',
    #     arguments=['compressed', 'in:=/left_camera/image', 'raw', 'out:=/left_camera/image'],
    #     output='screen',
    #     respawn=True
    # )
    
    return LaunchDescription([
        rviz_arg,
        lidar_slam_node #,
        # rviz_node #,
        # republish_node
    ])
