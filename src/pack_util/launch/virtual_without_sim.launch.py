from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('pack_util'), 'config', 'config_params.yaml')
    slam_param_path = os.path.join(get_package_share_directory('pack_util'), 'config', 'mapper_params_online_async.yaml')

    slam_toolbox = Node(package='slam_toolbox', node_executable='async_slam_toolbox_node', node_name='slam_toolbox', parameters=[slam_param_path])
    yolo_node = Node(package='pack_yolo', node_executable='yolo_node', node_name='yolo_node', parameters=[param_path])
    odom_node = Node(package='pack_qcar', node_executable='odom_node', node_name='odom_node', parameters=[param_path])
    pose_node = Node(package='pack_qcar', node_executable='pose_node', node_name='pose_node', parameters=[param_path])

    return LaunchDescription([yolo_node, odom_node, pose_node])

