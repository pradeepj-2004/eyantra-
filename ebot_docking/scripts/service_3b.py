#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Replace with your package name and service type
import math, statistics

flag=[0]

class MyServiceServer(Node):
    def __init__(self):
        super().__init__('my_service_server')

        self.robot_pose = [0.0, 0.0, 0.0]
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.srv = self.create_service(DockSw, 'docking_service', self.service_callback)
        

    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range

    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range

    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw

    def send_velocity_command(self, ang, lin):
        msg=Twist()
        msg.angular.z=ang
        msg.linear.x=lin
        self.vel_publisher.publish(msg)

    def controller_loop(self):
        # d=(self.usrright_value-self.usrleft_value)
        # b = 0.3 
        # theta1= math.atan(d/b)
        # theta = theta1*180/3.14
        kpl = 0.5
        kpy = 0.02
        theta=(1.57-self.robot_pose[2])*(180/3.14)
        print(theta)
        if theta <= 1.0 and theta >=-1.0:   
            velocity=kpl*0.5*(self.usrright_value+self.usrleft_value)
            
            # if velocity <= 0.0001515:
            #     self.send_velocity_command(0.0,0.0)
            #     flag[0]=1
            # else :
            #     self.send_velocity_command(-velocity,0.0)
                
        else: 
            self.send_velocity_command(-theta*kpy,0.0)

    # def odometry_loop(self):
    #     yaw = self.robot_pose[2]
    #     print(yaw)
    #     kpy = 0.9
    #     e_yaw = -1.57 -yaw
    #     omega = kpy*e_yaw
    #     if (e_yaw <=0.05 and e_yaw >= -0.05):
    #         self.send_velocity_command(0.0,0.0)
    #         print("reached to desired oriantation")
            
    #     else:
    #         self.send_velocity_command(0.0,omega)
  


    def service_callback(self, request, response):

        self.docking_completed = False
        linear = request.linear_dock
        orientation = request.orientation_dock
        
        if orientation == True and linear == True:
            self.controller_loop()
            if flag[0]==1:
                response.success = True
                response.message = "Docked"
            else:
                response.success = False
                response.message = "Docked"

            

        if linear == False and orientation == True:
            print("i am here")
            self.controller_timer = self.create_timer(0.1, self.check_odom_controller_loop)
            response.success = True
            response.message = "Docked"
            print("response sended")

        return response
    

def main(args=None):
    rclpy.init(args=args)

    node = MyServiceServer()
    node.get_logger().info("node started")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()