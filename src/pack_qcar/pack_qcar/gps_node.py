#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import PoseStamped

from pal.products.qcar import QCarGPS

from .submodules.geometry import get_quaternion_from_euler



class GPSNode(Node):

    def __init__(self):
        super().__init__(
            'gps_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = 1,
                history 	  = 1,
                durability    = 1,
                depth 		  = 1)
        
        self.gps = QCarGPS()
        self.checkGPSTimer = self.create_timer(1/60, self.gps_callback)
        self.gpsPub = self.create_publisher(PoseStamped, '/gps', self.qcar_qos_profile)


    def gps_callback(self):
        if self.gps.read():
            orientation = get_quaternion_from_euler(self.gps.orientation[0], self.gps.orientation[1], self.gps.orientation[2])
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.position.x = float(self.gps.position[0])
            msg.pose.position.y = float(self.gps.position[1])
            msg.pose.orientation.x = float(orientation[0])
            msg.pose.orientation.y = float(orientation[1])
            msg.pose.orientation.z = float(orientation[2])
            msg.pose.orientation.w = float(orientation[3])
            self.gpsPub.publish(msg)
    
    def __del__(self):
        self.gps.terminate()


def main(args=None):
    rclpy.init(args=args)
    
    r = GPSNode()
    rclpy.spin(r)
    
    r.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
	main()
