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
from geometry_msgs.msg import TransformStamped
import time
from tf2_ros.buffer import Buffer
from tf2_ros import TransformException
from tf2_ros.transform_listener import TransformListener

import tf2_ros

class FrameListener(Node):

    def __init__(self):
        
        super().__init__('sample_tf2_frame_listener') 
        self.tf_buffer = Buffer()                                                                       # initializing transform buffer object
        self.tf_listener = TransformListener(self.tf_buffer, self)                                      # initializing transform listner object
        self.timer = self.create_timer(0.01, self.on_timer)  
        self.br = tf2_ros.TransformBroadcaster(self)                                             #Creating timer for updating tool0 tf value
        # self.srv = self.create_service(Tfvalue, 'my_service_pradeep', self.service_callback)            #Creating service so that we can get updated tf_value of tool0 like feedback

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
            self.br.sendTransform(tf_msg)

        except Exception as e:
                self.get_logger().error(f"Error looking up or publishing transform: {str(e)}")
            

def main():
    rclpy.init()
    node = FrameListener()
    try:
        print("Publisher started")
        rclpy.spin(node)                                                                                  #Spinning node continously to update Tf listener value
    except KeyboardInterrupt:
        print("Publisher ended")
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
