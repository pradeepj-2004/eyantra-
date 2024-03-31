#!/usr/bin/env python3

'''
# Team ID:          2083
# Theme:            Cosmo Logistic
# Author List:      Santosh, Pradeep, Mohit, Parth
# Filename:         ebot_task3b.py
# Functions:        attach_callback, box_number, rack_pose, rpl_to_quaternion, detach_callback, set_value_fun, dock_callback, callback, send_velocity_command, linear_shift_backside
# Global variables: None 
'''


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ebot_docking.srv import DockSw
import math
import time
from linkattacher_msgs.srv import AttachLink, DetachLink
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler
from example_interfaces.srv import AddTwoInts


import yaml
from os import path
import os


yaml_file = path.join(
    path.dirname(path.realpath(__file__)), "rack_pose.yaml"
)

need=[0]

class MyEbotNav(Node):
    def __init__(self):
        super().__init__('my_service')

        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')
        self.dock = self.create_client(DockSw, 'dock_control')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.odom=self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        self.rack_pub=self.create_client(AddTwoInts,'arm_starter')
        
        self.robot_pose = [0.0, 0.0, 0.0]

        self.rack_no = None
        self.rack_home_pose = None
        self.rack_drop_pose = None

        self.dock_angle_home=None
        self.dock_angle_drop=None
    
    def send_goal(self,msg):
        goal_msg = AddTwoInts.Request()
        goal_msg.a = msg
        return self.rack_pub.call_async(goal_msg)

    def box_number(self):

        with open(yaml_file, 'r') as file:
            rack_pose = yaml.safe_load(file)

        i=rack_pose["package_id"]
        self.rack_no = i
    
    def rack_pose(self, i):
    
        with open(yaml_file, 'r') as file:
            rack_pose = yaml.safe_load(file)
        s=f"rack"+str(i)
        pose=rack_pose["position"][i-1][s]
        x=pose[0]
        y=pose[1]
        z = 0.0
        theta=pose[2]
        pose_drop=rack_pose["drop"][i-1][s]
        x_drop = pose_drop[0]
        y_drop = pose_drop[1]
        z_drop = 0.0
        yaw_drop = pose_drop[2]
        
        self.dock_angle_home=theta
        self.dock_angle_drop=yaw_drop
        
        self.quaternion=quaternion_from_euler(0.0, 0.0, theta)

        self.rack_home_pose = [x, y, z, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]]
        self.quaternion=quaternion_from_euler(0.0, 0.0, yaw_drop)

        self.rack_drop_pose = [x_drop, y_drop, z_drop, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3] ]


    def attach_callback(self, model1_name, model2_name, link1_name, link2_name):

        request= AttachLink.Request()
        request.model1_name = model1_name
        request.model2_name = model2_name
        request.link1_name = link1_name
        request.link2_name = link2_name
        self.link_attach_cli.call_async(request)

        self.get_logger().info("Attached")
        
    def detach_callback(self, model1_name, model2_name, link1_name, link2_name):

        request= DetachLink.Request()
        request.model1_name = model1_name
        request.model2_name = model2_name
        request.link1_name = link1_name
        request.link2_name = link2_name
        self.link_detach_cli.call_async(request)

        self.get_logger().info("Detached")

    def dock_callback(self, linear_dock, orientation_dock, docking_angle):
    
        request= DockSw.Request()
        request.linear_dock = linear_dock
        request.orientation_dock = orientation_dock
        request.dock_angle=docking_angle
        future = self.dock.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        print("response received")

    def send_velocity_command(self, v, w):
        
        msg=Twist()
        msg.angular.z=w
        msg.linear.x=v
        self.vel_publisher.publish(msg)
    
    def odometry_callback(self, msg):
        
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        list_1=[self.robot_pose[0],self.robot_pose[1]]
        if need[0]==1:
            self.future.set_result(list_1)
    
    def linear_shift(self,val,sign):
        need[0]=1
        prev_value=self.wait_for_callback_result()

        while True:
            odom=self.wait_for_callback_result()
            if (math.sqrt((odom[0]-prev_value[0])**2 + (odom[1]-prev_value[1])**2)<val):
                self.send_velocity_command(0.1*sign,0.0)
            else:
                self.send_velocity_command(0.0,0.0)
                print('done linear correction')
             
                break
        need[0]=0
    
    def wait_for_callback_result(self):
        self.future=rclpy.Future()
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    

def main(args=None):
    
    rclpy.init(args=args)

    client = MyEbotNav()

    client.get_logger().info('client created')
    i = 0
    navigator = BasicNavigator() #starting basic navigator
    
    while not client.rack_pub.wait_for_service(timeout_sec=1.0):
        client.get_logger().warn('rack service not available, waiting again...')
    future = client.send_goal(1)                                                #Sending request for service which will respond with tool 0 tf_value
    rclpy.spin_until_future_complete(client, future)
    
    while i < 2:

        client.box_number() #getting box number
    
        rack_no_pre = client.rack_no[i] #rack number in integer
        rack_no = f"rack"+str(rack_no_pre) #rack number in string form
        client.rack_pose(rack_no_pre) # getting rack pose

        rhp = client.rack_home_pose #getting rack home pose
        rdp = client.rack_drop_pose #getting rack drop pose
        print(rhp) # printing rack home pose
        print(rdp) #printing rack drop pose

        if i == 0:
            initial_pose = PoseStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = navigator.get_clock().now().to_msg()
            initial_pose.pose.position.x = 0.0
            initial_pose.pose.position.y = 0.0
            initial_pose.pose.position.z = 0.05
            initial_pose.pose.orientation.x = 0.0
            initial_pose.pose.orientation.y = 0.0
            initial_pose.pose.orientation.z = 0.0
            initial_pose.pose.orientation.w = 1.0
            navigator.setInitialPose(initial_pose) #seting initial pose of bot 
            
            navigator.waitUntilNav2Active()
        
        goal_poses1 = []
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rhp[0]
        goal_pose.pose.position.y = rhp[1] 
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = rhp[3]
        goal_pose.pose.orientation.y = rhp[4]
        goal_pose.pose.orientation.z = rhp[5]
        goal_pose.pose.orientation.w = rhp[6]
        goal_poses1.append(goal_pose) #pre dock pose 

        goal_poses2 = []

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = rdp[0]
        goal_pose.pose.position.y = rdp[1]
        goal_pose.pose.position.z = 0.0 
        # goal_pose.pose.orientation.x = rdp[3]
        # goal_pose.pose.orientation.y = rdp[4]
        # goal_pose.pose.orientation.z = rdp[5]
        # goal_pose.pose.orientation.w = rdp[6]

        goal_poses2.append(goal_pose)


        navigator.followWaypoints(goal_poses1) #this will move ebot to near to the rack 
        while not navigator.isTaskComplete():
            time.sleep(1)

        client.dock_callback(True, True, client.dock_angle_home+math.pi) #start docking

        client.attach_callback("ebot",rack_no,"ebot_base_link","link") #attaching rack

        client.linear_shift(0.3,1)
        
        os.system('ros2 param set /local_costmap/local_costmap footprint "[ [0.2, 0.4], [0.2, -0.4], [-0.2, -0.4], [-0.2, 0.4] ]"') #updating robot size
                
        navigator.followWaypoints(goal_poses2) # this will take the ebot to the droping location 

        while not navigator.isTaskComplete(): #waiting to complete task
                time.sleep(1)


        client.dock_callback(False,True, client.dock_angle_home+math.pi) # docking at the drop pose 

        # if i==0:
        client.linear_shift(0.48,-1)
        # else: 
        #     client.linear_shift(0.45,-1)
        
        client.detach_callback("ebot",rack_no,"ebot_base_link","link") #detaching rack
        os.system('ros2 param set /local_costmap/local_costmap footprint "[ [0.2, 0.185], [0.2, -0.185], [-0.20, -0.185], [-0.2, 0.185] ]"') #updating size of the bot 
        
        client.linear_shift(0.3,1)
        
        while not client.rack_pub.wait_for_service(timeout_sec=1.0):
            client.get_logger().warn('rack service not available, waiting again...')
        future = client.send_goal(i+2)                                                #Sending request for service which will respond with tool 0 tf_value
        rclpy.spin_until_future_complete(client, future)
        i = i+1
     

if __name__ == '__main__':
    main()