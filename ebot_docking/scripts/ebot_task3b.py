#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Range
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from tf_transformations import euler_from_quaternion
from ebot_docking.srv import DockSw  # Import custom service message
import math, statistics
import time
from linkattacher_msgs.srv import AttachLink, DetachLink
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration 
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml


class ebot_dock(Node):
    def __init__(self):
        super().__init__('ebot_node')
        self.robot_pose = [0.0, 0.0, 0.0]
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odometry_callback, 10)
        self.link_attach_cli = self.create_client(AttachLink, '/ATTACH_LINK')
        self.link_detach_cli = self.create_client(DetachLink, '/DETACH_LINK')
        self.dock = self.create_client(DockSw, 'docking_service')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)


    def update_robot_size(self, yaml_file, new_size):
        with open(yaml_file, 'r') as file:
            data = yaml.safe_load(file)

        data['local_costmap']['local_costmap']['ros__parameters']['footprint'] = new_size

        with open(yaml_file, 'r') as file:
            yaml.dump(data, file, default_flow_style = False)
    

    def odometry_callback(self, msg):
        self.robot_pose[0] = msg.pose.pose.position.x
        self.robot_pose[1] = msg.pose.pose.position.y
        quaternion_array = msg.pose.pose.orientation
        orientation_list = [quaternion_array.x, quaternion_array.y, quaternion_array.z, quaternion_array.w]
        _, _, yaw = euler_from_quaternion(orientation_list)
        self.robot_pose[2] = yaw


    def send_velocity_command(self, v, w):
        msg=Twist()
        msg.angular.z=w
        msg.linear.x=v
        self.vel_publisher.publish(msg)
    
    def linear_shift_backside(self, distance):
        t = distance/0.15
        self.send_velocity_command(-0.15, 0.0)
        time.sleep(t)
        self.send_velocity_command(0.0, 0.0)

    def attach_callback(self):
        request= AttachLink.Request()
        request.model1_name = 'ebot'
        request.model2_name = 'rack3'
        request.link1_name = 'ebot_base_link'
        request.link2_name = 'link'
        self.link_attach_cli.call_async(request)

        self.get_logger().info("Attached")
        rate = self.create_rate(2, self.get_clock())
        
    def detach_callback(self):
        request= DetachLink.Request()
        request.model1_name = 'ebot'
        request.model2_name = 'rack3'
        request.link1_name  = 'ebot_base_link'
        request.link2_name  = 'link'
        self.link_detach_cli.call_async(request)

        self.get_logger().info("Detached")
        rate = self.create_rate(2, self.get_clock())

    def dock_callback(self, linear_dock, orientation_dock):
        request= DockSw.Request()
        request.linear_dock = linear_dock
        request.orientation_dock = orientation_dock
        # self.get_logger().info("Docked")
        future = self.dock.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)

    client = ebot_dock()
    client.get_logger().info('client created')
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
    goal_pose.pose.position.x = 2.2
    goal_pose.pose.position.y = -7.5
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.7068252
    goal_pose.pose.orientation.w = 0.7073883
    goal_poses1.append(goal_pose)

    goal_poses2 = []

    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 1.2
    goal_pose.pose.position.y = -3.555 
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = 0.0
    goal_pose.pose.orientation.y = 0.0
    goal_pose.pose.orientation.z = 0.7068252 
    goal_pose.pose.orientation.w = 0.7073883

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

    navigator.followWaypoints(goal_poses1)
    client.get_logger().info("goalpose1 started")
    while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
    client.get_logger().info("goalpose1 finished")

    while True:
            while not client.dock.wait_for_service(timeout_sec=1.0):
                client.get_logger().warn('dock service not available, waiting again...')
            
            future = client.dock_callback(True, True)
            rclpy.spin_until_future_complete(client, future)
            print(future.result().success)
    
    exit()
    time.sleep(15)

    client.attach_callback()

    navigator.followWaypoints(goal_poses2)
    client.get_logger().info("goalpose2 started")
    while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
    client.get_logger().info("goalpose2 finished")

    client.dock_callback(False,True)

    time.sleep(15)

    client.linear_shift_backside(0.25)

    client.detach_callback()

    navigator.followWaypoints(goal_poses3)
    client.get_logger().info("goalpose3 started")
    while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
    client.get_logger().info("goalpose3 finished")

    navigator.followWaypoints(goal_poses4)
    client.get_logger().info("goalpose4 started")
    while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()
    client.get_logger().info("goalpose3 finished")
    
    navigator.lifecycleShutdown() 

if __name__ == '__main__':
    main()
