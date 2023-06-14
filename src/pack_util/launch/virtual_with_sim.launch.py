from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('pack_util'), 'config', 'config_params.yaml')

    qcar_node = Node(package='pack_qcar', node_executable='qcar_node', node_name='qcar_node', parameters=[param_path])
    rgbd_node = Node(package='pack_qcar', node_executable='rgbd_node', node_name='rgbd_node', parameters=[param_path])
    gps_node = Node(package='pack_qcar', node_executable='gps_node', node_name='gps_node', parameters=[param_path])
    lidar_node = Node(package='pack_qcar', node_executable='lidar_node', node_name='lidar_node', parameters=[param_path])
    
    # gamepad_node = Node(package='pack_qcar', node_executable='gamepad_node', node_name='gamepad_node', parameters=[param_path])
    # objectposition_node = Node(package='pack_qcar', node_executable='objectposition_node', node_name='objectposition_node', parameters=[param_path])

    return LaunchDescription([qcar_node, gps_node, odom_node, lidar_node, pose_node, rgbd_node])

