#!/usr/bin/env python3

'''
# Team ID:          < 2083 >
# Theme:            < Cosmo Logistic >
# Author List:      < Pradeep,Mohit,Santosh,Parth >
# Filename:         < tf_box >
# Functions:        < __init__,on_timer,service_callback>
# Global variables: <>
'''

#This is service created for getting tf value of tool0 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped,TransformStamped

from tf2_ros import TransformException


from ur5_moveit.srv import Tfvalue

import tf2_ros

class FrameListener(Node):

    def __init__(self):
        
        super().__init__('sample_tf2_frame_listener') 
                                          # initializing transform listner object
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.on_timer)                                             #Creating timer for updating tool0 tf value
        self.srv = self.create_service(Tfvalue, 'my_service_pradeep', self.service_callback)            #Creating service so that we can get updated tf_value of tool0 like feedback

    def on_timer(self):
        try:
            # Look up the transform between 'base_link' and 'obj_<marker_id>'
            transform = self.tf_buffer.lookup_transform('base_link', 'tool0', rclpy.time.Time())
            tf_msg = TransformStamped()
            tf_msg.header.stamp = self.get_clock().now().to_msg()
            tf_msg.header.frame_id = 'base_link'
            tf_msg.child_frame_id = 'tool0_1'
            tf_msg.transform.translation = transform.transform.translation
            tf_msg.transform.rotation=transform.transform.rotation
        except TransformException as e:
            print("wait")
            return
            
        self.l1=tf_msg.transform.translation.x
        self.l2=tf_msg.transform.translation.y
        self.l3=tf_msg.transform.translation.z
        self.l4=tf_msg.transform.rotation.x
        self.l5=tf_msg.transform.rotation.y
        self.l6=tf_msg.transform.rotation.z
        self.l7=tf_msg.transform.rotation.w
        
    def service_callback(self,request,response):                                                        #Service callback
        
        response.tx=self.l1                 
        response.ty=self.l2
        response.tz=self.l3
        response.rx=self.l4
        response.ry=self.l5
        response.rz=self.l6
        response.rw=self.l7
        print('response sent')
        return response
        

def main():
    rclpy.init()
    node = FrameListener()
    try:
        print("Service started")
        rclpy.spin(node)                                                                                  #Spinning node continously to update Tf listener value
    except KeyboardInterrupt:
        print("Service ended")
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
