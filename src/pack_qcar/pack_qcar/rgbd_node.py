#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from pal.products.qcar import QCarRealSense

from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import numpy as np
import cv2
import time
from skimage import transform
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar Realsense Node

class RGBDNode(Node):
    def __init__(self):
        super().__init__(
        'rgbd_node',
        allow_undeclared_parameters=True,
        automatically_declare_parameters_from_overrides=True
        )
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.rgbd_color_resolution_width = self.get_parameter('rgbd_color_resolution_width').value
        self.rgbd_color_resolution_height = self.get_parameter('rgbd_color_resolution_height').value
        self.rgbd_depth_resolution_width = self.get_parameter('rgbd_depth_resolution_width').value
        self.rgbd_depth_resolution_height = self.get_parameter('rgbd_depth_resolution_height').value
        self.rgbd_color_freq = self.get_parameter('rgbd_color_freq').value
        self.rgbd_depth_freq = self.get_parameter('rgbd_depth_freq').value

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = QoSReliabilityPolicy.BEST_EFFORT,
                history 	  = QoSHistoryPolicy.KEEP_LAST,
                durability    = QoSDurabilityPolicy.VOLATILE,
                depth 		  = 10)
        
        self.bridge = CvBridge()
        # self.tform = np.array([[ 1.28390668e+00, -7.05919459e-02, -9.06734264e+01], [-1.51282171e-02,  1.33292367e+00, -9.17617539e+01], [-9.65535577e-05, -2.64383441e-04,  1.00000000e+00]])
        # self.tform = np.array([[ 1.15935701e+00, -4.03841113e-02, -8.29815980e+01], [-1.41326970e-02,  1.30028745e+00, -9.70338124e+01], [-1.87811097e-04, -5.21489683e-04,  1.00000000e+00]])
        self.tform = np.array([[ 1.29283048e+00,  6.89161088e-02, -1.16665723e+02], [-5.22919851e-02,  1.29774728e+00, -7.73933518e+01], [-1.86623461e-04, -9.98066423e-05,  1.00000000e+00]])
        self.tform = transform.ProjectiveTransform(self.tform)
        
        self.imageColorPublisher = self.create_publisher(CompressedImage, '/qcar/rgbd_color', self.qcar_qos_profile)
        self.imageColor	= QCarRealSense(mode='RGB', frameWidthRGB=self.rgbd_color_resolution_width, frameHeightRGB=self.rgbd_color_resolution_height, frameRateRGB=self.rgbd_color_freq)
        self.imageColorTimer = self.create_timer(1/self.rgbd_color_freq, self.rgbd_color_callback)

        self.imageDepthPublisher = self.create_publisher(Image, '/qcar/rgbd_depth', self.qcar_qos_profile)
        self.imageDepth	= QCarRealSense(mode='DEPTH', frameWidthDepth=self.rgbd_depth_resolution_width, frameHeightDepth=self.rgbd_depth_resolution_height, frameRateDepth=self.rgbd_depth_freq)
        self.imageDepthTimer = self.create_timer(1/self.rgbd_depth_freq, self.rgbd_depth_callback)
        
        # self.imageColorDepthTimer = self.create_timer(1/self.rgbd_depth_freq, self.mycallback)

    def mycallback(self):
        self.imageColor.read_RGB()
        rgb_img = self.imageColor.imageBufferRGB
        self.imageDepth.read_depth(dataMode='PX')
        depth_img = self.imageDepth.imageBufferDepthPX        
        
        tf_img = transform.warp(depth_img, self.tform.inverse)
        tf_img = np.interp(tf_img, (0, 1), (0, 255)).astype(np.uint8)
        
        cv2.imshow("RGB", rgb_img)
        cv2.imshow("Depth", depth_img)
        cv2.imshow("tf_img", tf_img)
        cv2.waitKey(1)

    def rgbd_color_callback(self):
        self.imageColor.read_RGB()
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        msg.data = np.array(cv2.imencode('.jpg', self.imageColor.imageBufferRGB)[1]).tostring()
        msg.format = "jpg"
        self.imageColorPublisher.publish(msg)
        
        # cameraInfo = self.imageColorPublisher
        # # read IntelRealsense color information based on camera settings
        # self.imageColor.read_RGB()
        # # Conversion and publishing camera image infromation
        # publishImage	= self.bridge.cv2_to_compressed_imgmsg(self.imageColor.imageBufferRGB, dst_format = "jpg")
        # publishImage.header.stamp    =  self.get_clock().now().to_msg()
        # publishImage.header.frame_id = 'RGBD_color_input'
        # cameraInfo.publish(publishImage)

    def rgbd_depth_callback(self):
        # cameraInfo = self.imageDepthPublisher

        # # read IntelRealsense Depth information based on camera settings
        # self.imageDepth.read_depth(dataMode='PX')
        # print(self.imageDepth.imageBufferDepthPX)

        # # Conversion and publishing camera image infromation
        # publishImage	= self.bridge.cv2_to_imgmsg(self.imageDepth.imageBufferDepthPX,
        #                                                                 "8UC1")

        # publishImage.header.stamp 	 =  self.get_clock().now().to_msg()
        # publishImage.header.frame_id = 'RGBD_depth_input'
        # cameraInfo.publish(publishImage)
        # return
        # cameraInfo = self.imageDepthPublisher
        # self.imageDepth.read_depth(dataMode='m')
        # cvim = self.imageDepth.imageBufferDepthM
        # encoding = "32FC1"
        # img_msg = Image()
        # img_msg.height = cvim.shape[0]
        # img_msg.width = cvim.shape[1]
        # img_msg.encoding = encoding
        # img_msg.data = cvim.tobytes()
        # img_msg.step = len(img_msg.data) // img_msg.height
        # img_msg.header.stamp 	 =  self.get_clock().now().to_msg()
        # img_msg.header.frame_id = 'RGBD_depth_input'
        # cameraInfo.publish(img_msg)
        
        # cameraInfo = self.imageDepthPublisher
        # # read IntelRealsense Depth information based on camera settings
        # self.imageDepth.read_depth(dataMode='m')
        # # Conversion and publishing camera image infromation
        # publishImage	= self.bridge.cv2_to_imgmsg(self.imageDepth.imageBufferDepthM, "32FC1")
        # publishImage.header.stamp 	 =  self.get_clock().now().to_msg()
        # publishImage.header.frame_id = 'RGBD_depth_input'
        # cameraInfo.publish(publishImage)
        
        self.imageDepth.read_depth(dataMode='PX')
        img = self.imageDepth.imageBufferDepthPX
        img = transform.warp(img, self.tform.inverse)
        img = np.interp(img, (0, 1), (0, 255)).astype(np.uint8)

        msg = Image()
        msg.height, msg.width = img.shape[0], img.shape[1]
        msg.data = img.ravel().tobytes()
        msg.step = len(msg.data) # msg.height
        msg.header.stamp 	 =  self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'
        self.imageDepthPublisher.publish(msg)
            
    # calback for the parameters # TODO should be tested on real cars
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'rgbd_color_resolution_width':
                self.rgbd_color_resolution_width = param.value
                self.imageColor.terminate()
                self.imageColor	= QCarRealSense(mode='RGB', frameWidthRGB=self.rgbd_color_resolution_width, frameHeightRGB=self.rgbd_color_resolution_height, frameRateRGB=self.rgbd_color_freq)
            elif param.name == 'rgbd_color_resolution_height':
                self.rgbd_color_resolution_height = param.value
                self.imageColor.terminate()
                self.imageColor	= QCarRealSense(mode='RGB', frameWidthRGB=self.rgbd_color_resolution_width, frameHeightRGB=self.rgbd_color_resolution_height, frameRateRGB=self.rgbd_color_freq)
            elif param.name == 'rgbd_depth_resolution_width':
                self.rgbd_depth_resolution_width = param.value
                self.imageDepth.terminate()
                self.imageDepth	= QCarRealSense(mode='DEPTH', frameWidthDepth=self.rgbd_depth_resolution_width, frameHeightDepth=self.rgbd_depth_resolution_height, frameRateDepth=self.rgbd_depth_freq)
            elif param.name == 'rgbd_depth_resolution_height':
                self.rgbd_depth_resolution_height = param.value
                self.imageDepth.terminate()
                self.imageDepth	= QCarRealSense(mode='DEPTH', frameWidthDepth=self.rgbd_depth_resolution_width, frameHeightDepth=self.rgbd_depth_resolution_height, frameRateDepth=self.rgbd_depth_freq)
            elif param.name == 'rgbd_color_freq':
                self.rgbd_color_freq = param.value
                self.imageColor.terminate()
                self.imageColor	= QCarRealSense(mode='RGB', frameWidthRGB=self.rgbd_color_resolution_width, frameHeightRGB=self.rgbd_color_resolution_height, frameRateRGB=self.rgbd_color_freq)
                self.imageColorTimer.cancel()
                self.imageColorTimer = self.create_timer(1/self.rgbd_color_freq, self.rgbd_color_callback)
            elif param.name == 'rgbd_depth_freq':
                self.rgbd_depth_freq = param.value
                self.imageDepth.terminate()
                self.imageDepth	= QCarRealSense(mode='DEPTH', frameWidthDepth=self.rgbd_depth_resolution_width, frameHeightDepth=self.rgbd_depth_resolution_height, frameRateDepth=self.rgbd_depth_freq)
                self.imageDepthTimer.cancel()
                self.imageDepthTimer = self.create_timer(1/self.rgbd_depth_freq, self.rgbd_depth_callback)
        return SetParametersResult(successful=True)
    
    def stop_rgbd(self):
        print("stopping Intel RealSense Nodes please wait...")
        self.imageColor.terminate()
        self.imageDepth.terminate()
                
    def __del__(self):
        self.stop_rgbd()
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Main

def main(args=None):
    rclpy.init(args=args)

    r = RGBDNode()
    rclpy.spin(r)

    r.destroy_node()
    rclpy.shutdown()
    
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Run

if __name__ == '__main__':
	main()
#endregion
