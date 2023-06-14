#!/usr/bin/env python3
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : File Description and Imports

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import PoseStamped, Point, PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from pack_msgs.msg import YoloDetectionArray
from std_srvs.srv import Trigger
from datetime import datetime
import os
import csv

from .submodules.geometry import euler_from_quaternion, get_quaternion_from_euler

import matplotlib.pyplot as plt
from pytransform3d import rotations as pr
from pytransform3d import transformations as pt
from pytransform3d.transform_manager import TransformManager
# from sklearn.cluster import KMeans

import numpy as np
import cv2
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : QCar Drive Node

class DetectionNode(Node):

    def __init__(self):
        super().__init__(
            'detection_node',
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True
        )

        # test out making custom QoS profile:
        self.qcar_qos_profile = QoSProfile(
                reliability   = QoSReliabilityPolicy.BEST_EFFORT,
                history 	  = QoSHistoryPolicy.KEEP_LAST,
                durability    = QoSDurabilityPolicy.VOLATILE,
                depth 		  = 10)
        
        # self.fov_h = 60.518 # field of view - virtual
        self.fov_h = 29.05*2 # field of view - real
        self.fov_v = 58 # field of view
        self.px_2_angle = {px:((self.fov_h/2) - px*(self.fov_h/640)) for px in range(0, 640)} 

        # self.px_2_meter = {px:(0.1 + px*(9.9/256)) for px in range(0, 256)} # virtual
        self.px_2_meter = {px:(0.08 + px*(0.25)) for px in range(0, 256)} # real
        
        # nodes parameters (TODO add them to rosparam)
        self.scale = 1
        self.min_num_of_seen = 2
        self.close_enough = 0.5
        self.std_region_height = 10
        self.std_region_width = 10
        self.min_depth_to_detect = 0.3
        self.max_depth_to_detect = 2.0 # 8 for virtual
        self.std_threshold = 1.0
        self.car_yaw_limit_for_cross_walk = 10
        self.camera_angle_limit_for_cross_walk = 10
        
        self.position = None
        self.orientation = None
        self.depth_frame = None
        self.container = [] # list of tuples: (x, y, label, number_of_seen)
        
        self.object_2_color = {"big_cone": ColorRGBA(r=float(1.0), g=float(0.4), b=float(0.0), a=1.0), "traffic_light": ColorRGBA(r=float(0.0), g=float(1.0), b=float(0.0), a=1.0), "cross_walk": ColorRGBA(r=float(1.0), g=float(1.0), b=float(0.0), a=1.0), "yield": ColorRGBA(r=float(0.0), g=float(0.0), b=float(1.0), a=1.0), "small_cone": ColorRGBA(r=float(1.0), g=float(0.65), b=float(0.3), a=1.0), "parking": ColorRGBA(r=float(102/255), g=float(51/255), b=float(0.0), a=1.0), "roundabout": ColorRGBA(r=float(153/255), g=float(51/255), b=float(1), a=1.0), "line": ColorRGBA(r=float(1), g=float(1), b=float(1), a=1.0), "stop_sign": ColorRGBA(r=float(153/255), g=float(0), b=float(0), a=1.0)}
        self.desktop_path = os.path.join(os.path.join(os.path.expanduser('~')), 'Desktop')
        
        # Initialize Publishers and Subscribers
        self.srv = self.create_service(Trigger, "/objectposition/save", self.save)
        self.visalizationuPublisher = self.create_publisher(Marker, '/visualization_marker', 0)
        self.depthSubscriber = self.create_subscription(Image, '/qcar/rgbd_depth', self.process_depth, self.qcar_qos_profile)
        self.poseSubscriber = self.create_subscription(PoseStamped, '/pose_map', self.process_pose, self.qcar_qos_profile)
        self.subscription_yolo_detections = self.create_subscription(YoloDetectionArray, '/yolo_detection', self.detection_callback, self.qcar_qos_profile)
        self.subscription_yolo_detections = self.create_subscription(PointStamped, '/clicked_point', self.remove_point, self.qcar_qos_profile)

    def save(self, request, response):
        now = datetime.now()
        file_name = now.strftime("%d-%m-%Y_%H-%M-%S") + '.csv'
        file_path = os.path.join(self.desktop_path, file_name)
        with open(file_path, 'w', newline='') as file:
            writer = csv.writer(file)
            for item in self.container:
                writer.writerow([item[2], item[0], item[1]])
        response.success = True
        response.message = f"captured {file_name}"
        return response
        
    def remove_point(self, msg:PointStamped):
        x, y = msg.point.x, msg.point.y
        remove_index = None
        for i, item in enumerate(self.container):
            x_, y_ = item[0], item[1]
            if np.sqrt((x-x_)**2 + (y-y_)**2) <= self.close_enough:
                remove_index = i
                break
        if remove_index is not None:
            self.container = [item for i, item in enumerate(self.container) if i != remove_index]
            self.draw_all()
        
                
        
    def draw_all(self):
        # delete all
        # marker = Marker()
        # marker.header.frame_id = "map"
        # marker.header.stamp = self.get_clock().now().to_msg()
        # marker.ns = 'signs'
        # marker.id = 0
        # marker.action = Marker.DELETEALL
        # self.visalizationuPublisher.publish(marker)
        
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'signs'
        marker.id = 0
        marker.type = Marker.CUBE_LIST
        marker.action = Marker.MODIFY
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.y = 0.2
        marker.scale.x = 0.2
        marker.scale.z = 0.2
        for item in self.container:
            if item[3] < self.min_num_of_seen:
                continue
            x = item[0]
            y = item[1]
            label = item[2]
            marker.points.append(Point(x=float(x), y=float(y), z=0.0))
            marker.colors.append(self.object_2_color[label])
        self.visalizationuPublisher.publish(marker)


    def add(self, x, y, label):
        for i, item in enumerate(self.container):
            x_ = item[0]
            y_ = item[1]
            label_ = item[2]
            num_ = item[3]
            if label == label_:
                distance = np.sqrt((x-x_)**2 + (y-y_)**2)
                if distance < self.close_enough:
                    x_avg = (num_*x_ + x)/(num_+1)
                    y_avg = (num_*y_ + y)/(num_+1)
                    self.container[i] = (x_avg, y_avg, label, num_+1)
                    break
        else:
            self.container.append((x, y, label, 1))
            
    
    def detection_callback(self, msg):
        
        if self.position is None:
            return
        if self.depth_frame is None:
            return
        
        for detection in msg.yolo_detections:
            x, y = detection.x, detection.y
            region = self.depth_frame[y-self.std_region_height//2:y+self.std_region_height//2, x-self.std_region_width//2:x+self.std_region_width//2]
            std = np.std(region)
            depth = self.px_2_meter[int(self.depth_frame[y, x])]
            angle = self.px_2_angle[x]
            # print(detection.label, depth, std)
            
            # if detection.label == "traffic_light":
            #     continue
            if detection.label == "car":
                continue
            if depth > self.max_depth_to_detect or depth < self.min_depth_to_detect:
                continue
            if std > self.std_threshold or std is np.nan:
                if detection.label != "cross_walk":
                    continue
            yaw = self.orientation_euler[2] * 180 / np.pi
            if detection.label == 'cross_walk' or detection.label == 'traffic_light':
                x = self.car_yaw_limit_for_cross_walk
                if not (0-x < yaw < 0+x or 90-x < yaw < 90+x or -90-x < yaw < -90+x or 180-x < yaw < 180 or -180 < yaw < -180+x):
                    print("inside", yaw)
                    continue
                x = self.camera_angle_limit_for_cross_walk
                if angle < -x or angle > x:
                    continue
            x_p = depth
            y_p = x_p * np.tan(angle * np.pi / 180)
            # print(x_p, y_p, angle)
            
            car2map = pt.transform_from_pq([self.position.x*self.scale, self.position.y*self.scale, self.position.z*self.scale, self.orientation.w, self.orientation.x, self.orientation.y, self.orientation.z])
            camera2car = pt.transform_from_pq([0.115*self.scale, 0*self.scale, 0, 1, 0, 0, 0])
            object2camera = pt.transform_from_pq([x_p, y_p, 0, 1, 0, 0, 0])
            
            tm = TransformManager()
            tm.add_transform("car", "map", car2map)
            tm.add_transform("object", "camera", object2camera)
            tm.add_transform("camera", "car", camera2car)
            object2map = tm.get_transform("object", "map")
            orientation = pr.quaternion_from_matrix(object2map[0:3, 0:3])
            position = object2map[0:3, 3]
            
            # ax = tm.plot_frames_in("car", s=0.1)
            # plt.show()
            
            self.add(position[0]/self.scale, position[1]/self.scale, detection.label)
            # if detection.label == "cross_walk":
            #     self.add((position[0] + np.random.rand())/self.scale, (position[1] + np.random.rand())/self.scale, "traffic_light")
            if detection.label == "traffic_light":
                self.add((position[0] + np.random.rand()/10)/self.scale, (position[1] + np.random.rand()/10)/self.scale, "cross_walk")
        
        remove_indices_list = set()
        for i in range(len(self.container)):
            for j in range(i+1, len(self.container)):
                first = self.container[i]
                second = self.container[j]
                if first[2] == second[2]: # labels
                    x1 = first[0]
                    y1 = first[1]
                    x2 = second[0]
                    y2 = second[1]
                    if np.sqrt((x1-x2)**2 + (y1-y2)**2) <= self.close_enough:
                        remove_indices_list.add(i)
        self.container = [item for i, item in enumerate(self.container) if i not in remove_indices_list]
            
        # Kmeans
        # cross_walks = [(item[0], item[1]) for item in self.container if item[2] == 'cross_walk']
        # traffic_lights = [(item[0], item[1]) for item in self.container if item[2] == 'traffic_light']
        # if len(cross_walks) >= 4:
        #     kmeans = KMeans(n_clusters=4)
        #     kmeans.fit(cross_walks)
        #     self.container = list(filter(lambda item: item[2] != 'cross_walk', self.container))
        #     for center in kmeans.cluster_centers_:
        #         self.container.append((center[0], center[1], 'cross_walk', self.min_num_of_seen))
        # if len(traffic_lights) >= 4:
        #     kmeans = KMeans(n_clusters=4)
        #     kmeans.fit(traffic_lights)
        #     self.container = list(filter(lambda item: item[2] != 'traffic_light', self.container))
        #     for center in kmeans.cluster_centers_:
        #         self.container.append((center[0], center[1], 'traffic_light', self.min_num_of_seen))

        self.draw_all()


    def process_depth(self, msg):        
        flatten_array = np.frombuffer(msg.data, dtype=np.uint8)
        self.depth_frame = np.reshape(flatten_array, (msg.height, msg.width, 1))

        
        
    def process_pose(self, msg):
        self.position = msg.pose.position
        self.orientation = msg.pose.orientation
        self.orientation_euler = euler_from_quaternion(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w) 
          
           
    def __del__(self):
        pass
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : main
def main(args=None):
    rclpy.init(args=args)
    
    r = DetectionNode()
    rclpy.spin(r)
    
    r.destroy_node()
    rclpy.shutdown()
    
#endregion

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --

#region : run
if __name__ == '__main__':
	main()
#endregion
