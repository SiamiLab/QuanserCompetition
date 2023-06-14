#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult

from sensor_msgs.msg import CompressedImage
from pack_msgs.msg import YoloDetectionArray, YoloDetection

from .submodules import darknet

import os
from ctypes import *
import os
import cv2
import time
import numpy as np
from queue import Queue

class YoloNode(Node):

    def __init__(self):
        super().__init__(
            'yolo_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )
        
        # ros2 param handling
        self.set_parameters_callback(self.parameter_callback)
        self.threshold = self.get_parameter('threshold').value

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = QoSReliabilityPolicy.BEST_EFFORT,
                history 	  = QoSHistoryPolicy.KEEP_LAST,
                durability    = QoSDurabilityPolicy.VOLATILE,
                depth 		  = 10)
        
        resource_folder_path = os.path.join(get_package_share_directory('pack_yolo'), 'resource')

        self.weights_file = os.path.join(resource_folder_path, "yolov4-custom_last.weights")
        self.config_file = os.path.join(resource_folder_path, "yolov4-custom.cfg")
        self.data_file = os.path.join(resource_folder_path, "obj.data")
        data_file_content = f"""
            classes = 12
            train = {os.path.join(resource_folder_path, "train.txt")}
            valid = {os.path.join(resource_folder_path, "test.txt")}
            names = {os.path.join(resource_folder_path, "obj.names")}
            backup = {resource_folder_path}
        """
        with open(self.data_file, "w") as text_file:
            text_file.write(data_file_content)
        
        
        
        self.frame_queue = Queue()
        self.darknet_image_queue = Queue(maxsize=1)
        self.detections_queue = Queue(maxsize=1)
        self.fps_queue = Queue(maxsize=1)
        
        self.network, self.class_names, self.class_colors = darknet.load_network(
            self.config_file,
            self.data_file,
            self.weights_file,
            batch_size=1
        )
        self.darknet_width = darknet.network_width(self.network)
        self.darknet_height = darknet.network_height(self.network)
        
        self.video_width = 640
        self.video_height = 480

        self.currentFrame = None
        # Initialize Publishers and Subscribers
        self.subscription_rgb = self.create_subscription(CompressedImage, '/qcar/rgbd_color', self.rgb_image_callback, self.qcar_qos_profile)
        self.detectionPublisher = self.create_publisher(YoloDetectionArray, '/yolo_detection', self.qcar_qos_profile)
    
    def rgb_image_callback(self, msg):
        np_arr = np.fromstring(np.array(msg.data).tobytes(), np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (self.darknet_width, self.darknet_height), interpolation=cv2.INTER_LINEAR)
        
        img_for_detect = darknet.make_image(self.darknet_width, self.darknet_height, 3)
        darknet.copy_image_from_bytes(img_for_detect, frame_resized.tobytes())
        
        # detection
        darknet_image = img_for_detect
        prev_time = time.time()
        detections = darknet.detect_image(self.network, self.class_names, darknet_image, thresh=self.threshold)
        fps = int(1/(time.time() - prev_time))
        # darknet.print_detections(detections, False)
        darknet.free_image(darknet_image)
        
        detections_adjusted = []
        for label, confidence, bbox in detections:
            bbox_adjusted = self.convert2original(frame, bbox)
            detections_adjusted.append((str(label), confidence, bbox_adjusted))
        
        yolo_detection_array = YoloDetectionArray()
        yolo_detection_array.header.stamp = self.get_clock().now().to_msg()
        yolo_detection_array.header.frame_id = 'yolo_detections'
        for label, confidence, bbox in detections_adjusted:
            yolo_detection_array.yolo_detections.append(YoloDetection(x=int(bbox[0]), y=int(bbox[1]), width=int(bbox[2]), height=int(bbox[3]), confidence=float(confidence), label=label))
            self.detectionPublisher.publish(yolo_detection_array)


    def convert2relative(self, bbox):
        """
        YOLO format use relative coordinates for annotation
        """
        x, y, w, h  = bbox
        _height     = self.darknet_height
        _width      = self.darknet_width
        return x/_width, y/_height, w/_width, h/_height


    def convert2original(self, image, bbox):
        x, y, w, h = self.convert2relative(bbox)

        image_h, image_w, __ = image.shape

        orig_x       = int(x * image_w)
        orig_y       = int(y * image_h)
        orig_width   = int(w * image_w)
        orig_height  = int(h * image_h)

        bbox_converted = (orig_x, orig_y, orig_width, orig_height)

        return bbox_converted


    def convert4cropping(self, image, bbox):
        x, y, w, h = self.convert2relative(bbox)

        image_h, image_w, __ = image.shape

        orig_left    = int((x - w / 2.) * image_w)
        orig_right   = int((x + w / 2.) * image_w)
        orig_top     = int((y - h / 2.) * image_h)
        orig_bottom  = int((y + h / 2.) * image_h)

        if (orig_left < 0): orig_left = 0
        if (orig_right > image_w - 1): orig_right = image_w - 1
        if (orig_top < 0): orig_top = 0
        if (orig_bottom > image_h - 1): orig_bottom = image_h - 1

        bbox_cropping = (orig_left, orig_top, orig_right, orig_bottom)

        return bbox_cropping
    
    # calback for the parameters
    def parameter_callback(self, params):
        for param in params:
            if param.name == 'threshold':
                self.threshold = param.value
        return SetParametersResult(successful=True)
    
    def __del__(self):
        pass

def main(args=None):
    rclpy.init(args=args)
    
    r = YoloNode()
    rclpy.spin(r)
    
    r.destroy_node()
    rclpy.shutdown()
    

if __name__ == '__main__':
	main()
