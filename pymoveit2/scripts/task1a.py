#!/usr/bin/env python3
'''

Team ID:          [ CL#2083 ]
Author List:	  [ Mohit Kumar, Santosh, Pradeep, Parth ]
Filename:		  task1a.py
Functions:
			      [ calculate_rectangle_area(), detect_aruco(), __init__(), depthimagecb(), colorimagecb(), process_image(), publish_tf(), main() ]
Nodes:		      Add your publishing and subscribing node
                  Example:
			        Publishing Topics  - [ /tf ]
                  Subscribing Topics - [ /camera/aligned_depth_to_color/image_raw, /etc... ]
'''

import rclpy
import sys
import cv2
import math
import time
import os
import platform
import tf2_ros
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import CompressedImage, Image
from cv2 import aruco
import numpy as np


def calculate_rectangle_area(coordinates):
    

    length = math.sqrt( math.pow( coordinates[0][0] - coordinates[1][0] , 2 ) + math.pow( coordinates[0][1] - coordinates[1][1] , 2 ))
    bredth = math.sqrt( math.pow( coordinates[1][0] - coordinates[2][0] , 2 ) + math.pow( coordinates[1][1] - coordinates[2][1] , 2 ))

    area = length * bredth

    if length == bredth :
        width = bredth
    elif length > bredth :
        width = bredth
    else :
        width = length


    return area, width


def detect_aruco(image): 
    
    aruco_area_threshold = 1500
    cam_mat = np.array([[931.1829833984375, 0.0, 640.0], [0.0, 931.1829833984375, 360.0], [0.0, 0.0, 1.0]])
    dist_mat = np.array([0.0,0.0,0.0,0.0,0.0])
    size_of_aruco_m = 0.15
    center_aruco_list = []
    distance_from_rgb_list = []
    angle_aruco_list = []
    width_aruco_list = []
    ids = []
 
    ############ ADD YOUR CODE HERE ############

    
    parameters = aruco.DetectorParameters()
    dictionary = aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    areas=[]
    
    try:
        if image.shape[0] > 0 and image.shape[1] > 0:
            frame_np = np.array(image)
            gray_img = cv2.cvtColor(frame_np, cv2.COLOR_BGR2GRAY)
            detector = cv2.aruco.ArucoDetector(dictionary, parameters)
            corners, ids, rejected = detector.detectMarkers(gray_img)

            for i in range(len(ids)) :
                current_marker_corners = [corners[i]]
                center_coor = [corners[i][0][0][0]+(corners[i][0][2][0]-corners[i][0][0][0])/2, corners[i][0][0][1]-(corners[i][0][0][1]-corners[i][0][2][1])/2]
                ret = cv2.aruco.estimatePoseSingleMarkers(current_marker_corners, size_of_aruco_m, cameraMatrix=cam_mat, distCoeffs=dist_mat)
                rvec, tvec = ret[0][0, 0, :], ret[1][0, 0, :]
                center_aruco_list.append(center_coor)
                angle_aruco_list.append(rvec)
                area,width=calculate_rectangle_area(current_marker_corners[0][0])
                areas.append(area)
                width_aruco_list.append(width)

                if area>aruco_area_threshold:
                    cv2.aruco.drawDetectedMarkers(image,current_marker_corners)
                    cv2.putText(image,f"id={ids[i]}",(int(center_coor[0]),int(center_coor[1])),cv2.FONT_HERSHEY_PLAIN,2,(0,0,255),2,cv2.LINE_AA)
                    cv2.circle(image,(int(center_coor[0]),int(center_coor[1])), 5, (255,0,0), 5)
                    cv2.drawFrameAxes(image, cam_mat, dist_mat, rvec, tvec, length =1)
                    cv2.putText(image,f"center",(int(center_coor[0]),int(center_coor[1])),cv2.FONT_HERSHEY_PLAIN,1,(0,255,0),2,cv2.LINE_AA)        

            cv2.imshow("RGB_camera", image)

        else:
            print("Invalid image dimensions.")

        return center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,areas
    except Exception as e:
        print(e)

class aruco_tf(Node):
   
    def __init__(self):
        
        super().__init__('aruco_tf_publisher')                                          # registering node

        ############ Topic SUBSCRIPTIONS ############

        self.color_cam_sub = self.create_subscription(Image, '/camera/color/image_raw', self.colorimagecb, 10)
        self.depth_cam_sub = self.create_subscription(Image, '/camera/aligned_depth_to_color/image_raw', self.depthimagecb, 10)

        ############ Constructor VARIABLES/OBJECTS ############

        image_processing_rate = 0.2                                                     # rate of time to process image (seconds)
        self.bridge = CvBridge()                                                        # initialise CvBridge object for image conversion
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)                                    # object as transform broadcaster to send transform wrt some frame_id
        self.timer = self.create_timer(image_processing_rate, self.process_image)       # creating a timer based function which gets called on every 0.2 seconds (as defined by 'image_processing_rate' variable)
        # self.timer1=self.create_timer(0.01,self.on_timer_1)
        self.cv_image = None                                                            # colour raw image variable (from colorimagecb())
        self.depth_image = None                                                         # depth image variable (from depthimagecb())

    # def on_timer_1(self):
        
    #     try:
    #         # Look up the transform between 'base_link' and 'obj_<marker_id>'
    #         transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
    #         tf_msg = TransformStamped()
    #         tf_msg.header.stamp = self.get_clock().now().to_msg()
    #         tf_msg.header.frame_id = 'base_link'
    #         tf_msg.child_frame_id = 'tool0_1'
    #         tf_msg.transform.translation = transform.transform.translation
    #         tf_msg.transform.rotation=transform.transform.rotation
    #         self.br.sendTransform(tf_msg)

    #     except Exception as e:
    #             self.get_logger().error(f"Error looking up or publishing transform: {str(e)}")

    def depthimagecb(self, data):
       

        try:
            bgr_depth_camera = self.bridge.imgmsg_to_cv2(data)
            backtorgb_depth_camera = cv2.cvtColor(bgr_depth_camera,cv2.COLOR_BGR2RGB)
            self.depth_image=backtorgb_depth_camera
            cv2.waitKey(1)
        except:
            print("Not able to get image")



    def colorimagecb(self, data):
       
        try:
            bgr_RGB_camera = self.bridge.imgmsg_to_cv2(data)
            gray = cv2.cvtColor(bgr_RGB_camera, cv2.COLOR_BGR2GRAY)
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize = (10, 10))
            img = clahe.apply(gray)
            # blur = cv2.GaussianBlur(img,(5,5),0)
            backtorgb_RGB_camera = cv2.cvtColor(bgr_RGB_camera,cv2.COLOR_BGR2RGB)
            self.cv_image=backtorgb_RGB_camera
            cv2.waitKey(1)
        except:
            print("Not able to get image")    

    def process_image(self):
        
        
        sizeCamX = 1280
        sizeCamY = 720
        centerCamX = 640 
        centerCamY = 360
        focalX = 931.1829833984375
        focalY = 931.1829833984375


        try:
            center_aruco_list, distance_from_rgb_list, angle_aruco_list, width_aruco_list, ids,areas=detect_aruco(self.cv_image)
            
            for i in range(len(ids)):
                distance_from_rgb_list.append(center_aruco_list[i])
                xp=int(center_aruco_list[i][0])
                yp=int(center_aruco_list[i][1])

                depth_value=self.depth_image[yp,xp]
                c=depth_value[0]
        
                c=float(c/1000)
                distance_from_rgb_list.append(depth_value[0])

                x = c*(sizeCamX - center_aruco_list[i][0] - centerCamX) / focalX 
                y = c* (sizeCamY - center_aruco_list[i][1] - centerCamY) / focalY
                z = c
        

                r,p,ya = angle_aruco_list[i]
                
                r = 0.0
                p = 0.0
                ya = (0.788*ya) - ((ya**2)/3160)
                self.new_ya=ya
                
                cy = math.cos(ya * 0.5)
                sy = math.sin(ya * 0.5)
                cp = math.cos(p * 0.5)
                sp = math.sin(p * 0.5)
                cr = math.cos(r * 0.5)
                sr = math.sin(r * 0.5)

                w = cy * cp * cr + sy * sp * sr
                roll = cy * cp * sr - sy * sp * cr
                pitch = sy * cp * sr + cy * sp * cr
                yaw = sy * cp * cr - cy * sp * sr

                if areas[i]>1500:
                    self.publish_tf(z,x,y,roll,pitch,yaw,w,ids[i][0])
                    self.lookup_and_publish_transform(ids[i][0])
        
        except Exception as e:
            print(e, "in process fxn")
        

    def lookup_and_publish_transform(self,number2):
            try:
                transform = self.tf_buffer.lookup_transform('base_link', 'CL_2083_cam_'+str(number2), rclpy.time.Time())
            
                tf_msg = TransformStamped()
                r1=1.57
                p1=0.0
                y1=1.57-self.new_ya
                

                cy = math.cos(y1 * 0.5)
                sy = math.sin(y1 * 0.5)
                cp = math.cos(p1 * 0.5)
                sp = math.sin(p1 * 0.5)
                cr = math.cos(r1 * 0.5)
                sr = math.sin(r1 * 0.5)

                w2 = cy * cp * cr + sy * sp * sr
                r2 = cy * cp * sr - sy * sp * cr
                p2 = sy * cp * sr + cy * sp * cr
                y2 = sy * cp * cr - cy * sp * sr

                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'base_link'
                tf_msg.child_frame_id = 'CL_2083_base_'+str(number2)
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation.x = r2
                tf_msg.transform.rotation.y = p2
                tf_msg.transform.rotation.z = y2
                tf_msg.transform.rotation.w = w2
                
                self.br.sendTransform(tf_msg)

            except Exception as e:
                self.get_logger().info(f"Error looking up or publishing transform: {str(e)}")

    def publish_tf(self,X,Y,Z,R,P,YA,W,new):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = 'camera_link'
        tf_msg.child_frame_id = 'CL_2083_cam_'+str(new)
        tf_msg.transform.translation.x =X
        tf_msg.transform.translation.y =Y
        tf_msg.transform.translation.z =Z
        tf_msg.transform.rotation.w = W
        tf_msg.transform.rotation.x = R
        tf_msg.transform.rotation.y = P
        tf_msg.transform.rotation.z = YA


        self.br.sendTransform(tf_msg)


def main():
    '''
    Description:    Main function which creates a ROS node and spin around for the aruco_tf class to perform it's task
    '''

    rclpy.init(args=sys.argv)                                       # initialisation

    node = rclpy.create_node('aruco_tf_process')                    # creating ROS node

    node.get_logger().info('Node created: Aruco tf process')        # logging information

    aruco_tf_class = aruco_tf()                                     # creating a new object for class 'aruco_tf'

    rclpy.spin(aruco_tf_class)                                      # spining on the object to make it alive in ROS 2 DDS

    aruco_tf_class.destroy_node()                                   # destroy node after spin ends

    rclpy.shutdown()                                                # shutdown process


if __name__ == '__main__':
    main()