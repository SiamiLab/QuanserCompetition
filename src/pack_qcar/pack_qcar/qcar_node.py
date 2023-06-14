#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.parameter import Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import BatteryState, Imu

from pal.products.qcar import QCar

import numpy as np
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar Drive Node

class QCarNode(Node):

    def __init__(self):
        super().__init__(
            'qcar_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.is_simulation = self.get_parameter('is_simulation').value
        self.qcar_frequency = self.get_parameter('qcar_frequency').value

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = 1,
                history 	  = 1,
                durability    = 1,
                depth 		  = 1)

        # Initialize publisher data
        self.dataIMU 	 	= np.zeros((6,1))
        self.dataBattery 	= 0
        self.linearVelocity = 0

        # Initialize Publishers and Subscribers
        self.imuPublisher = self.create_publisher(Imu, '/qcar/imu', self.qcar_qos_profile)
        self.batteryPublisher = self.create_publisher(BatteryState, '/qcar/stateBattery', self.qcar_qos_profile)
        self.carVelocityPublisher = self.create_publisher(Vector3Stamped, '/qcar/velocity', self.qcar_qos_profile)
        self.commandSubscriber = self.create_subscription(Vector3Stamped, '/qcar/user_command', self.process_cmd, 1)
        
        # Initialize QCar Subscribers
        self.command   = np.array([0, 0])
        self.motor_cmd = np.array([0, 0])
        self.LEDs      = np.array([0, 0, 0, 0, 0, 0, 1, 1])
        # self.LEDs      = np.array([0, 0, 0, 0, 0, 0, 0, 0])

        # Configure QCar properties
        self.taskRate = int(self.qcar_frequency) #int(100) #Hz defualt: 500
        self.hardware = 1
        self.readMode = 1

        # Initialize QCar using task based I/O
        self.myCar = QCar(readMode=self.readMode, frequency=self.taskRate)
        
        # main loop frequency
        self.qcarTimer = self.create_timer(1/self.qcar_frequency, self.qcar_callback)

    def process_cmd(self, inputCommand):
        # Throttle and steering command from command publisher
        vel_cmd      = inputCommand.vector.x
        str_cmd      = inputCommand.vector.y
        self.command = np.array([vel_cmd, str_cmd])
        
        # Configure LEDs
        if str_cmd > 0.3:
            self.LEDs[0] = 1
            self.LEDs[2] = 1
        elif str_cmd < -0.3:
            self.LEDs[1] = 1
            self.LEDs[3] = 1
        if str_cmd < 0:
            self.LEDs[5] = 1
                  
    def qcar_callback(self):
        # writing commands
        self.myCar.read_write_std(throttle= self.command[0], steering= self.command[1], LEDs=self.LEDs)

        # reading sensors
        self.dataIMU = np.concatenate((self.myCar.accelerometer, self.myCar.gyroscope))
        self.dataBattery = self.myCar.batteryVoltage
        self.linearVelocity = self.myCar.motorTach
        
        # publish sensors
        self.publish_all_sensors()

    def publish_all_sensors(self):
        stamp = self.get_clock().now().to_msg()
        # Initialize Imu msg type
        stateIMU 					   = Imu()
        stateIMU.header.stamp 		   = stamp
        stateIMU.header.frame_id 	   = 'imu'
        stateIMU.angular_velocity.x    = float(self.dataIMU[3]) # attention: mistake from quanser (solved)
        stateIMU.angular_velocity.y    = float(self.dataIMU[4])
        stateIMU.angular_velocity.z    = float(self.dataIMU[5])
        stateIMU.linear_acceleration.x = float(self.dataIMU[0])
        stateIMU.linear_acceleration.y = float(self.dataIMU[1])
        stateIMU.linear_acceleration.z = float(self.dataIMU[2])
        self.imuPublisher.publish(stateIMU)
        
        # Initialize BatteryState msg type
        stateBattery                 = BatteryState()
        stateBattery.header.stamp    = stamp
        stateBattery.header.frame_id = 'battery_voltage'
        stateBattery.voltage         = float(self.dataBattery)
        self.batteryPublisher.publish(stateBattery)
        
        # Initialize Vector3Stamped msg type
        stateVelocity 				  = Vector3Stamped()
        stateVelocity.header.stamp    = stamp
        stateVelocity.header.frame_id = 'car_velocity'
        stateVelocity.vector.x 		  = float(np.cos(self.command[1]) * self.linearVelocity)
        stateVelocity.vector.y        = float(np.sin(self.command[1]) * self.linearVelocity)
        stateVelocity.vector.z        = float(self.linearVelocity) # we need linear velocity hence we publish it in 'z' eventhough its not z
        self.carVelocityPublisher.publish(stateVelocity)
    
    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'is_simulation':
                self.is_simulation = param.value
            elif param.name == 'qcar_frequency':
                self.qcar_frequency = param.value
                self.qcarTimer.cancel()
                self.qcarTimer = self.create_timer(1/self.qcar_frequency, self.qcar_callback)
        return SetParametersResult(successful=True)
    
    def stop_qcar(self):
        self.myCar.terminate()
        
    def __del__(self):
        self.stop_qcar()
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : main
def main(args=None):
    rclpy.init(args=args)
    
    r = QCarNode()
    rclpy.spin(r)
    
    r.destroy_node()
    rclpy.shutdown()
    
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : run
if __name__ == '__main__':
	main()
#endregion
