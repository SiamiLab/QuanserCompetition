#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tf2_msgs.msg import TFMessage 
from geometry_msgs.msg import TransformStamped, Vector3Stamped, PoseStamped
from sensor_msgs.msg import Imu

from hal.products.qcar import QCarEKF

from .submodules.geometry import euler_from_quaternion, get_quaternion_from_euler



class OdomNode(Node):
    def __init__(self):
        super().__init__(
            'odom_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = 1,
                history 	  = 1,
                durability    = 1,
                depth 		  = 1)
        
        self.ekf = QCarEKF(x_0=[0, 0, 0])
        
        self.tfPub = self.create_publisher(TFMessage, '/tf', self.qcar_qos_profile)
        self.tfStaticPub = self.create_publisher(TFMessage, '/tf_static', self.qcar_qos_profile)
        self.velocitySubscriber = self.create_subscription(Vector3Stamped, '/qcar/velocity', self.process_velocity, 1)
        self.velocitySubscriber = self.create_subscription(Imu, '/qcar/imu', self.process_imu, 1)
        self.commandSubscriber = self.create_subscription(Vector3Stamped, '/qcar/user_command', self.process_cmd, 1)        
        self.gpsSubscriber = self.create_subscription(PoseStamped, '/move_base_simple/goal', self.process_gps, 1) 
        
        self.motorTach = 0.0
        self.delta = 0.0
        self.gyroscope = [0.0, 0.0, 0.0]
        self.gps_position = [0.0, 0.0, 0.0]
        self.gps_orientation = [0.0, 0.0, 0.0]
        self.new_gps = False
        self.time_prev = time.time()

        # for bicycle model
        self.position = [0, 0]
        self.theta = 0
        self.t_prev = time.time()
        
         
        # calculate tf_static message
        basefootprint_baselink = TransformStamped()
        basefootprint_baselink.header.stamp = self.get_clock().now().to_msg()
        basefootprint_baselink.header.frame_id = 'base_footprint'
        basefootprint_baselink.child_frame_id = 'base_link'
        basefootprint_baselink.transform.translation.x = 0.0
        basefootprint_baselink.transform.translation.y = 0.0
        basefootprint_baselink.transform.translation.z = 0.01
        basefootprint_baselink.transform.rotation.x = 0.0
        basefootprint_baselink.transform.rotation.y = 0.0
        basefootprint_baselink.transform.rotation.z = 0.0
        basefootprint_baselink.transform.rotation.w = 1.0
        
        baselink_laserlink = TransformStamped()
        baselink_laserlink.header.stamp = self.get_clock().now().to_msg()
        baselink_laserlink.header.frame_id = 'base_link'
        baselink_laserlink.child_frame_id = 'laser_link' # 'lidar_1_link'
        baselink_laserlink.transform.translation.x = 0.0
        baselink_laserlink.transform.translation.y = 0.0
        baselink_laserlink.transform.translation.z = 0.19
        baselink_laserlink.transform.rotation.x = 0.0
        baselink_laserlink.transform.rotation.y = 0.0
        baselink_laserlink.transform.rotation.z = 0.0
        baselink_laserlink.transform.rotation.w = 1.0
        
        self.tf_static_msg = TFMessage()
        self.tf_static_msg.transforms = [basefootprint_baselink, baselink_laserlink]
    
    def process_cmd(self, msg):
        self.delta = float(msg.vector.y)
        
    def process_imu(self, msg):
        self.gyroscope = [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z]
        
    def process_gps(self, msg):
        self.gps_position = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
        euler = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.gps_orientation = [euler[0], euler[1], euler[2]]
        self.new_gps = True
        
    def process_velocity(self, msg):
        v = float(msg.vector.z)
        x_d = float(msg.vector.x)
        y_d = float(msg.vector.y)
        zita = self.delta + 0.06

        L = 0.256
        lr = L/2
        beta = np.arctan(lr * (np.tan(zita) / L))
        theta_d = v * (np.tan(zita) * np.cos(beta) / L)

        delta_t = time.time() - self.t_prev
        self.t_prev = time.time()

        self.theta = self.theta + theta_d*delta_t

        x_d = v * np.cos(self.theta + beta)
        y_d = v * np.sin(self.theta + beta)
        # x_d = float(msg.vector.x)
        # y_d = float(msg.vector.y)
        print(beta, zita)

        self.position[0] = self.position[0] + x_d * delta_t
        self.position[1] = self.position[1] + y_d * delta_t

        orientation = get_quaternion_from_euler(0, 0, self.theta)

        odom_basefootprint = TransformStamped()
        odom_basefootprint.header.stamp = self.get_clock().now().to_msg()
        odom_basefootprint.header.frame_id = 'odom'
        odom_basefootprint.child_frame_id = 'base_footprint'
        odom_basefootprint.transform.translation.x = self.position[0]
        odom_basefootprint.transform.translation.y = self.position[1]
        odom_basefootprint.transform.translation.z = 0.0
        odom_basefootprint.transform.rotation.x = float(orientation[0])
        odom_basefootprint.transform.rotation.y = float(orientation[1])
        odom_basefootprint.transform.rotation.z = float(orientation[2])
        odom_basefootprint.transform.rotation.w = float(orientation[3])
        
        tf_msg = TFMessage()  
        tf_msg.transforms = [odom_basefootprint]
        self.tfPub.publish(tf_msg)
        self.tfStaticPub.publish(self.tf_static_msg)


        # # quanser approach
        # self.motorTach = float(msg.vector.z)
        
        # time_current = time.time()
        # dt = time_current - self.time_prev
        # self.time_prev = time_current
        
        # if self.new_gps:
        #     self.new_gps = False
        #     y_gps = np.array([
        #         self.gps_position[0],
        #         self.gps_position[1],
        #         self.gps_orientation[2] - np.pi/2
        #     ])
        #     self.ekf.update(
        #         [self.motorTach, self.delta],
        #         dt,
        #         y_gps,
        #         self.gyroscope[2],
        #     )
        # else:
        #     self.ekf.update(
        #         [self.motorTach, self.delta + 0.06],
        #         dt,
        #         None,
        #         self.gyroscope[2],
        #     )
        # x = self.ekf.x_hat[0,0]
        # y = self.ekf.x_hat[1,0]
        # th = self.ekf.x_hat[2,0]

        # # x = self.gps_position[0]
        # # y = self.gps_position[1]
        # # th = self.gps_orientation[2] - np.pi/2
        # print("**", th * 180 / np.pi)
        
        # orientation = get_quaternion_from_euler(0, 0, th)
        # odom_basefootprint = TransformStamped()
        # odom_basefootprint.header.stamp = self.get_clock().now().to_msg()
        # odom_basefootprint.header.frame_id = 'odom'
        # odom_basefootprint.child_frame_id = 'base_footprint'
        # odom_basefootprint.transform.translation.x = x
        # odom_basefootprint.transform.translation.y = y
        # odom_basefootprint.transform.translation.z = 0.0
        # odom_basefootprint.transform.rotation.x = float(orientation[0])
        # odom_basefootprint.transform.rotation.y = float(orientation[1])
        # odom_basefootprint.transform.rotation.z = float(orientation[2])
        # odom_basefootprint.transform.rotation.w = float(orientation[3])
        
        # tf_msg = TFMessage()  
        # tf_msg.transforms = [odom_basefootprint]
        # self.tfPub.publish(tf_msg)
        # self.tfStaticPub.publish(self.tf_static_msg)
    
    
def main(args=None):  
    rclpy.init(args=args)

    r = OdomNode()
    rclpy.spin(r)

    r.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
	main()
