#!/usr/bin/env python3

## Overview

# ###
# This ROS2 script is designed to control a robot's docking behavior with a rack. 
# It utilizes odometry data, ultrasonic sensor readings, and provides docking control through a custom service. 
# The script handles both linear and angular motion to achieve docking alignment and execution.
# ###

# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        self.get_logger().info("Node started")
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Subscribe to ultrasonic sensor data for distance measurements
        self.ultrasonic_rl_sub = self.create_subscription(Range, '/ultrasonic_rl/scan', self.ultrasonic_rl_callback, 10)
        # Add another one here
        self.ultrasonic_rr_sub = self.create_subscription(Range, '/ultrasonic_rr/scan', self.ultrasonic_rr_callback, 10)

        #imu subscriber
        self.imu_sub= self.create_subscription(Imu,"/imu",self.imu_callback,10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'dock_control', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)


        # Initialize all  flags and parameters here
        self.is_docking = False
        self.linear_dock= False
        self.angular_dock = False

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)


    # Callback function for the left ultrasonic sensor
    def ultrasonic_rl_callback(self, msg):
        self.usrleft_value = msg.range
        
    # Callback function for the right ultrasonic sensor
    def ultrasonic_rr_callback(self, msg):
        self.usrright_value = msg.range
    
    def imu_callback(self,msg):
        
        self.imu_angle=euler_from_quaternion([msg.orientation.x,msg.orientation.y,msg.orientation.z,msg.orientation.w])
        self.yaw_value=self.normalize_angle(self.imu_angle[2])
        # print(self.yaw_value)
        
    # Utility function to normalize angles within the range of 0 to 2π (OPTIONAL)
    def normalize_angle(self,angle):
        angle=angle+math.pi
        return angle
        
    
    def send_velocity_command(self, v, w):
        msg=Twist()
        msg.linear.x=v
        msg.angular.z=w
        self.vel_publisher.publish(msg)

    # Main control loop for managing docking behavior

    def controller_loop(self):

        # The controller loop manages the robot's linear and angular motion 
        # control to achieve docking alignment and execution
        if self.is_docking:
            distance = (self.usrleft_value+self.usrright_value)/2
            kp_linear=0.4
            velocity=kp_linear*distance
            kp_angular=1.0
                         
            if (self.angular_dock==True and self.linear_dock==True):
                
                if distance <= 0.15 and self.turn==False:
                    self.send_velocity_command(0.0,0.0)
                    self.is_docking=False
                    print("linear docking done")
                elif (self.turn==False):
                    print('linear velocity:',velocity,"distance:",distance)
                    self.send_velocity_command(-velocity,0.0)
                
                if (self.turn==True):
                    angle_rad=self.dock_angle-self.yaw_value

                    if (abs(angle_rad)>math.pi):
                        if (self.yaw_value>math.pi):
                            angle_rad=2*math.pi-self.yaw_value+self.dock_angle
                            omega=kp_angular*(2*math.pi-self.yaw_value+self.dock_angle)
                        elif(self.yaw_value<math.pi):
                            angle_rad=2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle
                            omega=kp_angular*(2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle)

                    else:
                        omega=(kp_angular*angle_rad)

                    angle_diff=angle_rad*180/3.14

                    if (abs(angle_diff) >=2):
                        self.send_velocity_command(0.0,omega)
                        print('angular speed:',omega,' angle_diff:',angle_diff)
                    else:
                        print("angular docking done") 
                        self.send_velocity_command(0.0,0.0)
                        self.turn=False
         
            elif (self.angular_dock==True and self.linear_dock==False):
                angle_rad=self.dock_angle-self.yaw_value
  
                if (abs(angle_rad)>math.pi):
                    if (self.yaw_value>math.pi):
                        angle_rad=2*math.pi-self.yaw_value+self.dock_angle
                        omega=kp_angular*(2*math.pi-self.yaw_value+self.dock_angle)
                    elif(self.yaw_value<math.pi):
                        angle_rad=2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle
                        omega=kp_angular*(2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle)
                    
                else:
                    omega=(kp_angular*angle_rad)
                
                angle_diff=angle_rad*180/3.14

                if (abs(angle_diff) >=2):
                    self.send_velocity_command(0.0,omega)
                    print('angular speed:',omega,' angle_diff:',angle_diff)
                else:
                    print("angular docking done") 
                    self.send_velocity_command(0.0,0.0)
                    self.is_docking=False

        elif self.is_docking==False:
            self.dock_aligned=True

    # Callback function for the DockControl service
    def dock_control_callback(self, request, response):
        # Extract desired docking parameters from the service request
        self.angular_dock= request.orientation_dock
        self.linear_dock=request.linear_dock
        self.dock_angle=request.dock_angle  
          
        # Reset flags and start the docking process
        self.is_docking = True
        self.dock_aligned=False
        self.turn=False

        if (self.angular_dock==True and self.linear_dock==True):
            self.turn=True
        else:
            self.turn=False

        # Log a message indicating that docking has started
        print("Docking started for dock angle ",self.dock_angle)

        # Create a rate object to control the loop frequency
        rate = self.create_rate(2, self.get_clock())
        
        # Wait until the robot is aligned for docking
        while not self.dock_aligned:
            rate.sleep()

        # Set the service response indicating success

        response.success = True
        response.message = "Docking control initiated"
        print('response sent')
        return response

# Main function to initialize the ROS2 node and spin the executor
def main(args=None):
    rclpy.init(args=args)

    my_robot_docking_controller = MyRobotDockingController()

    executor = MultiThreadedExecutor()
    executor.add_node(my_robot_docking_controller)

    executor.spin()

    my_robot_docking_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
