#!/usr/bin/env python3
'''
# Team ID:          < 2083 >
# Theme:            < Cosmo Logistic >
# Author List:      < Pradeep,Mohit,Santosh,Parth >
# Filename:         < Starter_service >
# Functions:        < __init__,service_callback>
# Global variables: <l1>
'''
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import time
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros import TFMessage
from ur5_moveit.srv import Ur5
from pymoveit2.robots import ur5

l1=[0]
class start_node(Node):

    def __init__(self):
        
        super().__init__('sample_tf2_frame_listener') 
        self.tf_buffer = Buffer()                                                                       # initializing transform buffer object
        self.tf_listener = TransformListener(self.tf_buffer, self)                                      # initializing transform listner object 
        self.srv = self.create_service(Ur5, '/my_service_starter', self.service_callback)

    def service_callback(self,request,response):
        if request.requesting=='Done':
            l1[0]=1
            
        else:
            l1[0]=l1[0]
        
        if l1[0]==0:
            response.responsing='No'
        else:
            response.responsing='Yes'
        
        print(l1)
        return response

def main():
    rclpy.init()
    node = start_node()
    try:
        print("Service started")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Service ended")
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()

