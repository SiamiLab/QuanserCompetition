from ament_index_python.packages import get_package_share_directory
from  launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    param_path = os.path.join(get_package_share_directory('pack_util'), 'config', 'config_params.yaml')
    slam_param_path = os.path.join(get_package_share_directory('pack_util'), 'config', 'mapper_params_online_async.yaml')

    qcar_node = Node(package='pack_qcar', node_executable='qcar_node', node_name='qcar_node', parameters=[param_path])
    gps_node = Node(package='pack_qcar', node_executable='gps_node', node_name='gps_node', parameters=[param_path])
    odom_node = Node(package='pack_qcar', node_executable='odom_node', node_name='odom_node', parameters=[param_path])
    lidar_node = Node(package='pack_qcar', node_executable='lidar_node', node_name='lidar_node', parameters=[param_path])
    pose_node = Node(package='pack_qcar', node_executable='pose_node', node_name='pose_node', parameters=[param_path])
    rgbd_node = Node(package='pack_qcar', node_executable='rgbd_node', node_name='rgbd_node', parameters=[param_path])
    imgviewer_node = Node(package='pack_qcar', node_executable='imgviewer_node', node_name='imgviewer_node', parameters=[param_path])
    slam_toolbox = Node(package='slam_toolbox', node_executable='async_slam_toolbox_node', node_name='slam_toolbox', parameters=[slam_param_path])

    return LaunchDescription([qcar_node, gps_node, odom_node, lidar_node, pose_node, slam_toolbox, rgbd_node, imgviewer_node])

