'''
# Team ID:          2083
# Theme:            Cosmo Logistic
# Author List:      Santosh, Pradeep, Mohit, Parth
# Filename:         ebot_task3b.py
# Functions:        attach_callback, detach_callback, dock_callback, update_robot_size, send_velocity_command, linear_shift_backside
# Global variables: None 
'''

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
# from rclpy.callback_groups import ReentrantCallbackGroup
# from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw 
import math, statistics
import time
# from linkattacher_msgs.srv import AttachLink, DetachLink
from usb_relay.srv import RelaySw
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
# from ebot_docking.msg import Ecustomtopic
import yaml
from os import path

yaml_file = path.join(
    path.dirname(path.realpath(__file__)), "rack_pose.yaml"
)

# yaml_file_robot_params = path.join(
#     path.dirname("Home/colcon_ws/src/ebot_nav2/params/nav2_params.yaml")
# )

# class Updatevalue(Node):
#     def __init__(self):
#         super().__init__('my_special_topic_updater')
#         self.docking_status_susbcription = self.create_subscription(Ecustomtopic, 'custom_topic', self.callback, 10)
#         self.received_message = None

#     def callback(self, msg):
#         self.get_logger().info('Docking_status: {}' .format(msg.docking_status))
#         self.received_message = msg.docking_status
#         # print(self.received_message)
#     def set_value_fun(self):
#         self.received_message = "try"

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')

        self.link_attach_dettach_cli = self.create_client(RelaySw, 'usb_relay_sw')
        self.dock = self.create_client(DockSw, 'my_service_dock')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.quaternion = None
        self.rack_no = None
        self.rack_home_pose = None
        self.rack_drop_pose = None

    def rack_pose(self):
        with open(yaml_file, 'r') as file:
            rack_pose = yaml.safe_load(file)

        i=rack_pose["package_id"][0]
        self.rack_no = i
        s=f"rack"+str(i)
        # print(s)
        pose=rack_pose["position"][i-1][s]
        x=pose[0]
        y=pose[1]
        z = 0.0
        theta=pose[2]
        # print(x)
        # print(y)
        # print(theta)

        pose_drop=rack_pose["drop"][i-1][s]
        # print(pose_drop)
        x_drop = pose_drop[0]
        y_drop = pose_drop[1]
        z_drop = 0.0
        yaw_drop = pose_drop[5]
        # print(yaw_drop)
        self.rpl_to_quaternion(0.0, 0.0, yaw_drop)
        # print(self.quaternion)
        self.rack_home_pose = [x, y, z, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]]
        # print(self.rack_home_pose)
        self.rpl_to_quaternion(0.0,0.0, theta)
        self.rack_drop_pose = [x_drop, y_drop, z_drop, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3] ]


    def rpl_to_quaternion(self, roll, pitch, yaw):

        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr
        w = cy * cp * cr + sy * sp * sr

        self.quaternion = [qx, qy, qz, w]

    # def attach_callback(self):
    #     '''
    #     Purpose:
    #     ---
    #     this is used to attach the rack to the ebot

    #     Input Arguments:
    #     ---
    #     model1_name
    #     model2_name
    #     link1_name
    #     link2_name

    #     Returns:
    #     ---
    #     this will going to attach the rack and give the feed back that rack is attached or not 

    #     Example call:
    #     ---
    #     we can call it from another function or from main function

    #     '''
    #     request= RelaySw.Request()
    #     request.relaychannel = 0
    #     request.relaystate = True
    #     self.link_attach_dettach_cli.call_async(request)

    #     self.get_logger().info("Attached")
    #     rate = self.create_rate(2, self.get_clock())
        
    # def detach_callback(self):
    #     '''
    #     Purpose:
    #     ---
    #     this is used to detach the rack to the ebot

    #     Input Arguments:
    #     ---
    #     model1_name
    #     model2_name
    #     link1_name
    #     link2_name

    #     Returns:
    #     ---
    #     this will going to detach the rack and give the feed back that rack is detached or not 

    #     Example call:
    #     ---
    #     we can call it from another function or from main function

    #     '''
    #     request= RelaySw.Request()
    #     request.relaychannel = 0
    #     request.relaystate = False
    #     self.link_attach_dettach_cli.call_async(request)

    #     self.get_logger().info("Attached")
    #     rate = self.create_rate(2, self.get_clock())

    # def dock_callback(self, linear_dock, orientation_dock, rack_no):
    #     '''
    #     Purpose:
    #     ---
    #     this function is going to make request to the service to do docking and it will give the responce back

    #     Input Arguments:
    #     ---
    #     linear_dock
    #     this input is used to find that the ebot needed linear dock or not

    #     orientation_dock:
    #     this argument define the ebot needs the oriantational dock or not

    #     Returns:
    #     ---
    #     future(responce) 

    #     Example call:
    #     ---
    #     we can call it from main function

    #     '''
    #     request= DockSw.Request()
    #     request.linear_dock = linear_dock
    #     request.orientation_dock = orientation_dock
    #     request.rack_no = rack_no
    #     self.get_logger().info("Docked")
    #     rate = self.create_rate(2, self.get_clock())
    #     future = self.dock.call_async(request)
    #     return future

    # def update_robot_size(self, yaml_file, new_size):
    #     '''
    #     Purpose:
    #     ---
    #     this function is used to update the size paremeters inside the yaml file that is present in the ebot_nav2 directory so that after gatting

    #     Input Arguments:
    #     ---
    #     yaml_file
    #     this is the path of the yaml file which we need to update 

    #     new_size
    #     the new size of ebot according to the ebot having or not the rack

    #     Returns:
    #     ---
    #     None 

    #     Example call:
    #     ---
    #     we can call it from another function or from main function

    #     '''
    #     with open(yaml_file_robot_params, 'r') as file:
    #         data = yaml.safe_load(file)

    #     data['local_costmap']['local_costmap']['ros__parameters']['footprint'] = new_size

    #     with open(yaml_file, 'r') as file:
    #         yaml.dump(data, file, default_flow_style = False)

    # # def send_velocity_command(self, v, w):
    # #     '''
    # #     Purpose:
    # #     ---
    # #     To publish the velecity of the ebot 

    # #     Input Arguments:
    # #     ---
    # #     v  
    # #     v is the linear velocity of the ebot 

    # #     w 
    #     w is the angular velocity of the ebot

    #     Returns:
    #     ---
    #     It will going to publish this velocitys to the ebot and we will get a moving ebot

    #     Example call:
    #     ---
    #     we can call it whenever we need to publish velocity to the ebot

    #     '''
    #     msg=Twist()
    #     msg.angular.z=w
    #     msg.linear.x=v
    #     self.vel_publisher.publish(msg)
    
    # def linear_shift_backside(self, distance):
    #     '''
    #     Purpose:
    #     ---
    #     this function going to move the ebot in back side to a fixed distance

    #     Input Arguments:
    #     ---
    #     distance 
    #     thsi input we are going to give this is nothing but the distance is has to cover

    #     Returns:
    #     ---
    #     None

    #     Example call:
    #     ---
    #     we can call it whenever we need move our ebot in backward direction

    #     '''
    #     t = distance/0.1
    #     self.send_velocity_command(-0.1, 0.0)
    #     time.sleep(t)
    #     self.send_velocity_command(0.0, 0.0)

    

def main(args=None):

    '''
    Purpose:
    ---
    this function is used to perform the task to move the ebot near to the rack, dock, attach rack to the ebot, move to droping location, detach rack from the ebot, and againg move the ebot to the intial location  
    
    Input Arguments:
    ---
    None
    
    Returns:
    ---
    None
    
    Example call:
    ---
    Called automatically by the Operating System
    '''

    rclpy.init(args=args)

    client = MyService()
    # msg_updater = Updatevalue()
    client.get_logger().info('client created')
    client.rack_pose()
    rack_no_pre = client.rack_no
    rack_no = f"rack"+str(rack_no_pre)
    rhp = client.rack_home_pose
    rdp = client.rack_drop_pose


    navigator = BasicNavigator()
 
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
    navigator.setInitialPose(initial_pose)
    
    navigator.waitUntilNav2Active()
    
    goal_poses1 = []
    
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = rhp[0] 
    goal_pose.pose.position.y = rhp[1] 
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = rhp[3] #0.0
    goal_pose.pose.orientation.y = rhp[4] #0.0
    goal_pose.pose.orientation.z = -rhp[5] #0.7068252
    goal_pose.pose.orientation.w = rhp[6] #0.7073883
    goal_poses1.append(goal_pose)

    goal_poses2 = []

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 5.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.9999997
    goal_pose.pose.orientation.w = 0.0007

    goal_poses2.append(goal_pose)

    goal_poses3 = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.2
    goal_pose.pose.position.y = -4.4
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.9999997 
    goal_pose.pose.orientation.w = 0.0007963
    goal_poses3.append(goal_pose)

    goal_poses4 = []
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 0.0
    goal_pose.pose.position.y = 0.0
    goal_pose.pose.position.z = 0.0
    goal_poses4.append(goal_pose)

    nav_start = navigator.get_clock().now()

    navigator.followWaypoints(goal_poses1) #this will move ebot to near to the rack 
    while not navigator.isTaskComplete():
            print("goalpose1")
            feedback = navigator.getFeedback()

    # rack_no = client.rack_no

    # client.dock_callback(True, True, 3)

    # message = msg_updater.received_message
    # print(message)

    # while not message == 'docked':
    #     rclpy.spin_once(msg_updater)
    #     message = msg_updater.received_message
    #     print(message)
        # print("I am ready")
    # print("hello")

    # client.attach_callback()


    navigator.followWaypoints(goal_poses2) # this will take the ebot to the droping location 

    while not navigator.isTaskComplete():
            print("goalpose2")
            feedback = navigator.getFeedback()

    # client.dock_callback(False,True, 3)

    # message = msg_updater.received_message
    # print(message)
    # message = 'try'
    # msg_updater.set_value_fun()

    # while not message == 'docked':
    #     rclpy.spin_once(msg_updater)
    #     message = msg_updater.received_message
    #     print(message)
    #     # print("I am ready")
    # # print("hello")

    # client.linear_shift_backside(0.2)

    # client.detach_callback()


    navigator.followWaypoints(goal_poses3)
    while not navigator.isTaskComplete():
            print("goalpose3")
            feedback = navigator.getFeedback()

    navigator.followWaypoints(goal_poses4)
    while not navigator.isTaskComplete():
            print("goalpose4")
            feedback = navigator.getFeedback()


    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')
    
    navigator.lifecycleShutdown() #this will shout down the navigator
    exit()
    rclpy.shutdown()

if __name__ == '__main__':
    main()