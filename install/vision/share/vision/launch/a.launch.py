from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取包路径和配置文件路径
    pkg_share = get_package_share_directory('vision')
    config_dir = os.path.join(pkg_share, 'config')
    default_config = os.path.join(config_dir, 'a.yaml')
    
    # 声明配置文件参数
    config_path = LaunchConfiguration('config_path')
    config_arg = DeclareLaunchArgument(
        'config_path',
        default_value=default_config,
        description='Path to serial configuration YAML'
    )
    
    # 定义 publisher 节点
    publisher_node = Node(
        package='node',
        executable='publisher',
        name='publisher',
        output='screen',
        parameters=[config_path]
    )
    
    # 定义 subscriber 节点
    subscriber_node = Node(
        package='node',
        executable='subscriber',
        name='subscriber',
        output='screen',
        parameters=[config_path]
    )
    
    return LaunchDescription([
        config_arg,
        publisher_node,
        subscriber_node
    ])