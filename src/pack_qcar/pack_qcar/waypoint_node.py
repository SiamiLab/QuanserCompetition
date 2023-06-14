import signal
import numpy as np
import time

from pal.utilities.math import wrap_to_pi
from hal.products.mats import SDCSRoadMap

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Vector3Stamped, PoseStamped
from .submodules.geometry import euler_from_quaternion



class SpeedController:
    def __init__(self, kp=0, ki=0):
        self.maxThrottle = 0.3
        self.kp = kp
        self.ki = ki
        self.ei = 0

    # ==============  SECTION A -  Speed Control  ====================
    def update(self, v, v_ref, dt):
        e = v_ref - v
        self.ei += dt*e
        return np.clip(
            self.kp*e + self.ki*self.ei,
            -self.maxThrottle,
            self.maxThrottle
        )
        
class SteeringController:
    def __init__(self, waypoints, k=1, cyclic=True):
        self.maxSteeringAngle = np.pi/6
        self.wp = waypoints
        self.N = len(waypoints[0, :])
        self.wpi = 0
        self.k = k
        self.cyclic = cyclic
        self.p_ref = (0, 0)
        self.th_ref = 0

    # ==============  SECTION B -  Steering Control  ====================
    def update(self, p, th, speed):
        wp_1 = self.wp[:, np.mod(self.wpi, self.N-1)]
        wp_2 = self.wp[:, np.mod(self.wpi+1, self.N-1)]
        v = wp_2 - wp_1
        v_mag = np.linalg.norm(v)
        try:
            v_uv = v / v_mag
        except ZeroDivisionError:
            return 0
        tangent = np.arctan2(v_uv[1], v_uv[0])
        s = np.dot(p-wp_1, v_uv)
        if s >= v_mag:
            if  self.cyclic or self.wpi < self.N-2:
                self.wpi += 1
        ep = wp_1 + v_uv*s
        ct = ep - p
        dir = wrap_to_pi(np.arctan2(ct[1], ct[0]) - tangent)
        ect = np.linalg.norm(ct) * np.sign(dir)
        psi = wrap_to_pi(tangent-th)
        self.p_ref = ep
        self.th_ref = tangent
        return np.clip(
            wrap_to_pi(psi + np.arctan2(self.k*ect, speed)),
            -self.maxSteeringAngle,
            self.maxSteeringAngle)
        
        
        
class waypointNode(Node):

    def __init__(self):
        super().__init__(
            'waypoint_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.velocity_ref = self.get_parameter('velocity_ref').value
        self.velocity_kp = self.get_parameter('velocity_kp').value
        self.velocity_ki = self.get_parameter('velocity_ki').value
        self.steering_k_stanely = self.get_parameter('steering_k_stanely').value
        self.start_delay = self.get_parameter('start_delay').value
        
        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = 1,
                history 	  = 1,
                durability    = 1,
                depth 		  = 1)
        
        # self.nodeSequence = [13, 18, 10, 13]
        # self.nodeSequence = [13, 19, 15, 6, 23, 16, 11, 0, 4, 6, 10, 1]
        self.nodeSequence = [2, 4, 6, 8, 10, 1, 13, 19, 15, 6, 8, 23, 21, 16, 11, 8, 23, 21, 17, 11]
        self.nodeSequence = [2, 4, 6 ,8, 10, 1, 17, 20, 22, 9, 7, 14, 11, 8, 23, 21, 12, 0, 2]

        self.roadmap = SDCSRoadMap(leftHandTraffic=False)
        self.waypointSequence = self.roadmap.generate_path(self.nodeSequence)
        self.initialPose = self.roadmap.get_node_pose(self.nodeSequence[0]).squeeze()
        
        self.u = 0
        self.delta = 0

        self.speedController = SpeedController(kp=self.velocity_kp, ki=self.velocity_ki)
        self.steeringController = SteeringController(waypoints=self.waypointSequence, k=self.steering_k_stanely)

        
        self.t0 = time.time()
        self.t = 0
        
        self.position = [0.0, 0.0]
        self.yaw = 0
        self.linear_v = 0.0
        self.commandSubscriber = self.create_subscription(PoseStamped, '/pose', self.process_pose, 1)
        self.commandSubscriber = self.create_subscription(Vector3Stamped, '/qcar/velocity', self.process_velocity, 1)

        self.commandPublisher = self.create_publisher(Vector3Stamped, '/qcar/user_command', self.qcar_qos_profile)
        # self.carLoopTimer = self.create_timer(1/80, self.loop)

    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'velocity_ref':
                self.velocity_ref = param.value
            elif param.name == 'velocity_kp':
                self.velocity_kp = param.value
                self.speedController = SpeedController(kp=self.velocity_kp, ki=self.velocity_ki)
            elif param.name == 'velocity_ki':
                self.velocity_ki = param.value
                self.speedController = SpeedController(kp=self.velocity_kp, ki=self.velocity_ki)
            elif param.name == 'steering_k_stanely':
                self.steering_k_stanely = param.value
                self.steeringController = SteeringController(waypoints=self.waypointSequence, k=self.steering_k_stanely)
            elif param.name == 'start_delay':
                self.start_delay = param.value
                
        return SetParametersResult(successful=True)
    
    def process_velocity(self, msg):
        self.linear_v = msg.vector.z
        
    def process_pose(self, msg):
        self.position = [msg.pose.position.x, msg.pose.position.y]
        orientation = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        self.yaw = orientation[2]
        self.loop()
        
    def publish_commands(self, throttle, steering_angle):
        msg = Vector3Stamped()
        msg.vector.x = float(throttle)
        msg.vector.y = float(steering_angle)
        self.commandPublisher.publish(msg)
    
    def loop(self):
        tp = self.t
        self.t = time.time() - self.t0
        dt = self.t-tp
        
        x = self.position[0]
        y = self.position[1]
        th = self.yaw
        
        p = ( np.array([x, y]) + np.array([np.cos(th), np.sin(th)]) * 0.2)
        v = self.linear_v

        if self.t < self.start_delay:
            self.u = 0
            self.delta = 0
        else:
            self.u = self.speedController.update(v, self.velocity_ref, dt)
            self.delta = self.steeringController.update(p, th, v)

        # self.qcar.write(self.u, self.delta)
        self.publish_commands(self.u, self.delta)

        
    def __del__(self):
        pass
        # self.qcar.terminate()     
        
        
        
def main(args=None):  
    rclpy.init(args=args)

    r = waypointNode()
    rclpy.spin(r)

    r.destroy_node()
    rclpy.shutdown()



if __name__ == '__main__':
	main()