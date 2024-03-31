#!/usr/bin/env python3

'''
# Team ID:          < 2083 >
# Theme:            < Cosmo Logistic >
# Author List:      < Pradeep,Mohit,Santosh,Parth >
# Filename:         < tf_box >
# Functions:        < __init__,on_timer,service_callback>
# Global variables: <>
'''

#This is service created for arm starting

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

rack_no=[1]
class Service_class(Node):

    def __init__(self):
        
        super().__init__('arm_service')                                             #Creating timer for updating tool0 tf value
        self.srv = self.create_service(AddTwoInts, 'arm_starter', self.service_callback)            #Creating service so that we can get updated tf_value of tool0 like feedback
    
    def service_callback(self,request,response):                                                        #Service callback
        
        if (request.a==1):
            rack_no[0]=1
            print(rack_no[0],'published')
        if (request.a==2):
            rack_no[0]=2
            print(rack_no[0],'published')
        if (request.a==3):
            rack_no[0]=3
            print(rack_no[0],'published')
        
        response.sum=rack_no[0]         
        return response
        

def main():
    rclpy.init()
    node = Service_class()
    try:
        print("Service started")
        rclpy.spin(node)                                                                                  #Spinning node continously to update Tf listener value
    except KeyboardInterrupt:
        print("Service ended")
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
