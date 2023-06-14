#!/usr/bin/env python3

from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import PoseStamped, TransformStamped
from copy import deepcopy


class PoseNode(Node):
    def __init__(self):
        super().__init__(
            'pose_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(reliability = 1, history = 1, durability = 1, depth = 1)
        
        
        self.new_map_odom = False
        self.new_odom_basefootprint = False
        self.map_odom = TransformStamped()
        self.odom_basefootprint = TransformStamped()
        self.pose_mapPub = self.create_publisher(PoseStamped, '/pose_map', self.qcar_qos_profile)
        self.posePub = self.create_publisher(PoseStamped, '/pose', self.qcar_qos_profile)
        self.commandSubscriber = self.create_subscription(TFMessage, '/tf', self.process_tf, 1)


    def process_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == 'map' and transform.child_frame_id == 'odom':
                self.map_odom = deepcopy(transform)
                self.new_map_odom = True
            if transform.header.frame_id == 'odom' and transform.child_frame_id == 'base_footprint':
                self.odom_basefootprint = deepcopy(transform)
                self.new_odom_basefootprint = True
        if self.new_map_odom and self.new_odom_basefootprint:
            self.new_map_odom = False
            self.new_odom_basefootprint = False
            self.publish_pose()


    def publish_pose(self):
        odom2map = pt.transform_from_pq([self.map_odom.transform.translation.x, self.map_odom.transform.translation.y, self.map_odom.transform.translation.z, self.map_odom.transform.rotation.w, self.map_odom.transform.rotation.x, self.map_odom.transform.rotation.y, self.map_odom.transform.rotation.z])
        basefootprint2odom = pt.transform_from_pq([self.odom_basefootprint.transform.translation.x, self.odom_basefootprint.transform.translation.y, self.odom_basefootprint.transform.translation.z, self.odom_basefootprint.transform.rotation.w, self.odom_basefootprint.transform.rotation.x, self.odom_basefootprint.transform.rotation.y, self.odom_basefootprint.transform.rotation.z])

        tm = TransformManager()
        tm.add_transform("odom", "map", odom2map)
        tm.add_transform("basefootprint", "odom", basefootprint2odom)
        basefootprint2map = tm.get_transform("basefootprint", "map")
        # basefootprint2map = basefootprint2odom
        orientation = pr.quaternion_from_matrix(basefootprint2map[0:3, 0:3])
        position = basefootprint2map[0:3, 3]

        
        msg_posemap = PoseStamped()
        msg_posemap.header.stamp = self.get_clock().now().to_msg()
        msg_posemap.header.frame_id = 'map'
        msg_posemap.pose.position.x = float(position[0])
        msg_posemap.pose.position.y = float(position[1])
        msg_posemap.pose.orientation.x = float(orientation[1])
        msg_posemap.pose.orientation.y = float(orientation[2])
        msg_posemap.pose.orientation.z = float(orientation[3])
        msg_posemap.pose.orientation.w = float(orientation[0])
        
        msg_pose = PoseStamped()
        msg_pose.header.stamp = self.get_clock().now().to_msg()
        msg_pose.header.frame_id = 'map'
        msg_pose.pose.position.x = float(self.odom_basefootprint.transform.translation.x)
        msg_pose.pose.position.y = float(self.odom_basefootprint.transform.translation.y)
        msg_pose.pose.orientation.x = float(self.odom_basefootprint.transform.rotation.x)
        msg_pose.pose.orientation.y = float(self.odom_basefootprint.transform.rotation.y)
        msg_pose.pose.orientation.z = float(self.odom_basefootprint.transform.rotation.z)
        msg_pose.pose.orientation.w = float(self.odom_basefootprint.transform.rotation.w)
        
        self.pose_mapPub.publish(msg_posemap)
        self.posePub.publish(msg_pose)
       
       
    def __del__(self):
        pass


def main(args=None):
    rclpy.init(args=args)
    
    r = PoseNode()
    rclpy.spin(r)
    
    r.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
	main()
