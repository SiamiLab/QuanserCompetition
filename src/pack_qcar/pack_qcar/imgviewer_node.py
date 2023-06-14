#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

import numpy as np
import cv2
import time
from copy import deepcopy
import os
import platform
from datetime import datetime

from sensor_msgs.msg import Image, CompressedImage
from pack_msgs.msg import YoloDetectionArray, YoloDetection
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar ImageViewer Node

class ImageViewer(Node):
    def __init__(self):
        super().__init__('imgviewer_node',
        allow_undeclared_parameters=True,
		automatically_declare_parameters_from_overrides=True
	)
        self.set_parameters_callback(self.parameter_callback)
        self.mode = self.get_parameter('mode').value
        self.draw_yolo_detections = self.get_parameter('draw_yolo_detections').value

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = QoSReliabilityPolicy.BEST_EFFORT,
                history 	  = QoSHistoryPolicy.KEEP_LAST,
                durability    = QoSDurabilityPolicy.VOLATILE,
                depth 		  = 10)

        # start counter for time delta
        self.startTime = time.perf_counter()
        self.bridge    = CvBridge()
        self.mouseposition_txt = ''
        self.mouseposition = (-1, -1)
        self.yolo_detection_msg = None
        self.px_2_meter = {px:(0.1 + px*(9.9/256)) for px in range(0, 256)}
        
        self.currentFrame = None
        self.subscription_rgb = self.create_subscription(CompressedImage, '/qcar/rgbd_color', self.Compressed_Image_callback, self.qcar_qos_profile)
        self.subscription_depth = self.create_subscription(Image, '/qcar/rgbd_depth', self.Image_callback, self.qcar_qos_profile)
        self.subscription_yolo_detections = self.create_subscription(YoloDetectionArray, '/yolo_detection', self.detection_callback, self.qcar_qos_profile)
        
        self.desktop_path = os.path.join(os.path.join(os.environ['USERPROFILE']), 'Desktop') if platform.system() == 'Windows' else os.path.join(os.path.join(os.path.expanduser('~')), 'Desktop')
        self.srv = self.create_service(Trigger, '/qcar/take_photo', self.take_photo)
    
    def detection_callback(self, msg):
        if not self.draw_yolo_detections:
            self.yolo_detection_msg = None
        else:
            self.yolo_detection_msg = msg
    
    
    def take_photo(self, request, response):
        if self.currentFrame is None:
            response.success = False
            response.message = "No frame to capture"
            return response
        
        now = datetime.now()
        file_name = now.strftime("%d-%m-%Y_%H-%M-%S") + '.jpg'
        cv2.imwrite(os.path.join(self.desktop_path, file_name), self.currentFrame)
        response.success = True
        response.message = f"captured: {file_name}"
        return response
        
        
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'mode':
                self.mode = param.value
            elif param.name == 'draw_yolo_detections':
                self.draw_yolo_detections = param.value
        return SetParametersResult(successful=True)
    
    def Compressed_Image_callback(self,data):
        if self.mode.upper() != 'RGB':
            return  
        np_arr = np.fromstring(np.array(data.data).tobytes(), np.uint8)
        self.currentFrame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self.Image_display()

    def Image_callback(self, data):
        if self.mode.upper() != 'DEPTH':
            return
        # flatten_array = np.frombuffer(data.data, dtype=np.float32)
        flatten_array = np.frombuffer(data.data, dtype=np.uint8)
        self.currentFrame = np.reshape(flatten_array, (data.height, data.width, 1))
        self.Image_display()

    def Image_display(self):
        if self.currentFrame is None:
            return
        frame = deepcopy(self.currentFrame)
        time2 = time.perf_counter()
        # time from start of script or previous call
        timeDelta       = time2 - self.startTime
        framesPerSecond = str(1/timeDelta)
        # Image information:
        imageInfo = str(np.shape(frame))
        # text settings for open cv
        fpsCoordinate = (50,50)
        font          = cv2.FONT_HERSHEY_PLAIN
        fontScale     = 1
        color         = (255,0,0)
        thickness     = 2
        text  = framesPerSecond + str(" ")+ imageInfo + " " + f"({self.mouseposition[0]}, {self.mouseposition[1]})"
        if self.mode.upper() == "DEPTH":
            pxvalue = int(frame[self.mouseposition[1], self.mouseposition[0]])
            text += f" ({pxvalue}, {self.px_2_meter[pxvalue]})"
        image = cv2.putText(frame, text , fpsCoordinate, font, fontScale, color, thickness, cv2.LINE_AA )
        
        if self.yolo_detection_msg is not None:
            delay = (self.get_clock().now().nanoseconds - (self.yolo_detection_msg.header.stamp.sec*1e9 + self.yolo_detection_msg.header.stamp.nanosec)) / 1e9 # in seconds
            if delay < 1: 
                for detection in self.yolo_detection_msg.yolo_detections:
                    text = detection.label
                    textsize = cv2.getTextSize(text, font, fontScale, thickness)[0]
                    image = cv2.putText(image, text , (detection.x - textsize[0]//2, detection.y + textsize[1]//2 - detection.height//2 - 10), font, fontScale, color, thickness, cv2.LINE_AA )
                    image = cv2.circle(image, (detection.x, detection.y), 5, color, -1)
        cv2.imshow("Camera Video Stream", image)
        cv2.setMouseCallback('Camera Video Stream', self.click_event)
        cv2.waitKey(1)
        self.startTime = time2
    
    def click_event(self, event, x, y, flags, params):
  
        # checking for left mouse clicks
        if event == cv2.EVENT_MOUSEMOVE:
            self.mouseposition = (x-1, y-1)
            # self.mouseposition_txt = f'{x}, {y}'
            

#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Main

def main(args=None):
    rclpy.init(args=args)

    r = ImageViewer()
    rclpy.spin(r)

    r.destroy_node()
    rclpy.shutdown()
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : Run

if __name__ == '__main__':
    main()
#endregion
