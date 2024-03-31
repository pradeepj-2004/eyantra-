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
from geometry_msgs.msg import Twist

from ebot_docking.srv import DockSw 
import math
import time
from usb_relay.srv import RelaySw
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import yaml
from os import path
import os
from tf_transformations import quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray


from example_interfaces.srv import AddTwoInts


yaml_file = path.join(
    path.dirname(path.realpath(__file__)), "rack_pose.yaml"
)
#odometry
need=[0]

class MyService(Node):
    def __init__(self):
        super().__init__('my_service')

        self.link_attach_dettach_cli = self.create_client(RelaySw, 'usb_relay_sw')
        self.dock = self.create_client(DockSw, 'my_service_dock')
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)

        
        
        #odometry
        self.odom=self.create_subscription(Odometry, '/odometry/filtered', self.odometry_callback, 10)
        
        #rack_pose variable
        self.robot_pose = [0.0, 0.0, 0.0]
        self.quaternion = None
        self.rack_no = None
        self.rack_home_pose = None
        self.rack_drop_pose = None
        self.yaw_home = None
        self.yaw_drop = None
        
        self.rack_pub=self.create_client(AddTwoInts,'arm_starter')

    def ultra_callback(self,msg):
        self.usrleft_value= (msg.data[4])/100
        self.usrright_value = (msg.data[5])/100
    
    def send_goal(self,msg):
        goal_msg = AddTwoInts.Request()
        goal_msg.a = msg
        return self.rack_pub.call_async(goal_msg)

    def reset_odom(self):
        self.get_logger().info('Resetting Odometry. Please wait...')
        self.reset_odom_ebot = self.create_client(Trigger, 'reset_odom')
        while not self.reset_odom_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_odom service not available. Waiting for /reset_odom to become available.')

        self.request_odom_reset = Trigger.Request()
        self.odom_service_resp=self.reset_odom_ebot.call_async(self.request_odom_reset)
        rclpy.spin_until_future_complete(self, self.odom_service_resp)
        if(self.odom_service_resp.result().success== True):
            self.get_logger().info(self.odom_service_resp.result().message)
        else:
            self.get_logger().warn(self.odom_service_resp.result().message)

    def reset_imu(self):
        self.get_logger().info('Resetting IMU. Please wait...')
        self.reset_imu_ebot = self.create_client(Trigger, 'reset_imu')
        while not self.reset_imu_ebot.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('/reset_imu service not available. Waiting for /reset_imu to become available.')

        request_imu_reset = Trigger.Request()
        self.imu_service_resp=self.reset_imu_ebot.call_async(request_imu_reset)
        rclpy.spin_until_future_complete(self, self.imu_service_resp)
        if(self.imu_service_resp.result().success== True):
            self.get_logger().info(self.imu_service_resp.result().message)
        else:
            self.get_logger().warn(self.imu_service_resp.result().message)

    def box_number(self):
        with open(yaml_file, 'r') as file:
            rack_pose = yaml.safe_load(file)
        i=rack_pose["package_id"]
        self.rack_no = i

    def rack_pose(self,i):
        with open(yaml_file, 'r') as file:
            rack_pose = yaml.safe_load(file)

        s=f"rack"+str(i)
        pose=rack_pose["position"][i-1][s]
        x=pose[0]
        y=pose[1]
        z = 0.0
        self.yaw_home=pose[2]

        pose_drop=rack_pose["drop"][i-1][s]
        x_drop = pose_drop[0]
        y_drop = pose_drop[1]
        z_drop = 0.0
        self.yaw_drop = pose_drop[2]
        self.quaternion=quaternion_from_euler(0.0,0.0, self.yaw_home)
        
        self.rack_home_pose = [x, y, z, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3]]
        self.quaternion=quaternion_from_euler(0.0, 0.0, self.yaw_drop)
        self.rack_drop_pose = [x_drop, y_drop, z_drop, self.quaternion[0], self.quaternion[1], self.quaternion[2], self.quaternion[3] ]


    def switch_eletromagent(self,relayState):
        self.get_logger().info('Changing state of the relay to '+str(relayState))
        self.trigger_usb_relay = self.create_client(RelaySw, 'usb_relay_sw')
        while not self.trigger_usb_relay.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('USB Trigger Service not available, waiting...')

        request_relay = RelaySw.Request()
        request_relay.relaychannel = True
        request_relay.relaystate = relayState
        self.usb_relay_service_resp=self.trigger_usb_relay.call_async(request_relay)
        rclpy.spin_until_future_complete(self, self.usb_relay_service_resp)
        if(self.usb_relay_service_resp.result().success== True):
            self.get_logger().info(self.usb_relay_service_resp.result().message)
        else:
            self.get_logger().warn(self.usb_relay_service_resp.result().message)

    def dock_callback(self, linear_dock, orientation_dock, docking_angle):

        request= DockSw.Request()
        request.linear_dock = linear_dock
        request.orientation_dock = orientation_dock
        request.dock_angle  = docking_angle
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

    #odometry 
    def linear_shift(self,val,sign):
        need[0]=1
        prev_value=self.wait_for_callback_result()

        while True:
            odom=self.wait_for_callback_result()
            if (math.sqrt((odom[0]-prev_value[0])**2 + (odom[1]-prev_value[1])**2)<val):
                self.send_velocity_command(0.08*sign,0.0)
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

    client = MyService()
    # client.switch_eletromagent(False)
    client.get_logger().info('client created')
    # client.reset_odom()
    # client.reset_imu()
    i = 0
    navigator = BasicNavigator()
    
    #service part
    while not client.rack_pub.wait_for_service(timeout_sec=1.0):
        client.get_logger().warn('rack service not available, waiting again...')
    future = client.send_goal(1)         #Sending rackno. value
    rclpy.spin_until_future_complete(client, future)

    while i < 2:
        
        client.box_number()

        rack_no_pre = client.rack_no[i]
        client.rack_pose(rack_no_pre)
        
        # rack_no = f"rack"+str(rack_no_pre)
        rhp = client.rack_home_pose
        rdp = client.rack_drop_pose
        yaw_home_pose = client.yaw_home
        yaw_drop_pose = client.yaw_drop

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
        goal_pose.pose.position.x = rdp[0]#5.0
        goal_pose.pose.position.y = rdp[1]#0.0 
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.x = rdp[3]#0.0
        goal_pose.pose.orientation.y = rdp[4]
        goal_pose.pose.orientation.z = -rdp[5] 
        goal_pose.pose.orientation.w = rdp[6]

        goal_poses2.append(goal_pose)
        print(goal_poses1)

        navigator.followWaypoints(goal_poses1) #this will move ebot to near to the rack 
        while not navigator.isTaskComplete():
                time.sleep(1)

        print('navigation done near docking')
        client.switch_eletromagent(True)
        
        while not client.dock.wait_for_service(timeout_sec=1.0):
            client.get_logger().warn('docking service not available, waiting again...')

        client.dock_callback(True, True, yaw_home_pose)

        time.sleep(1)
        client.linear_shift(0.2,-1)
        time.sleep(1)
        # client.linear_shift(0.3,1)
        # distance = (client.usrleft_value+client.usrright_value)/2
        # i = 0
        # while i <=4:
        #     distance = (client.usrleft_value+client.usrright_value)/2
        #     if distance >= 0.3:    
        #         client.dock_callback(True, True, client.dock_angle_home+math.pi)
        #         if i == 1:
        #             print("hiiiii")
        #             client.linear_shift(0.3,1)
        #     else:
        #         break
            
        #     i = i+1

        os.system('ros2 param set /local_costmap/local_costmap footprint "[ [0.2, 0.4], [0.2, -0.4], [-0.2, -0.4], [-0.2, 0.4] ]"')

        navigator.followWaypoints(goal_poses2) # this will take the ebot to the droping location 

        while not navigator.isTaskComplete():
                time.sleep(1)
        
        while not client.dock.wait_for_service(timeout_sec=1.0):
            client.get_logger().warn('docking service not available, waiting again...')

        client.dock_callback(False,True, yaw_drop_pose)
        
        client.linear_shift(0.75,-1)
        os.system('ros2 param set /local_costmap/local_costmap footprint "[ [0.2, 0.185], [0.2, -0.185], [-0.20, -0.185], [-0.2, 0.185] ]"')


        client.switch_eletromagent(False)
        time.sleep(1)
        client.linear_shift(0.2,1)
        
        #service part
        while not client.rack_pub.wait_for_service(timeout_sec=1.0):
            client.get_logger().warn('rack service not available, waiting again...')
        future = client.send_goal(i+2)      #Sending rack_pose value
        rclpy.spin_until_future_complete(client, future)
        
        time.sleep(1)
        
        
        i = i+1
        

if __name__ == '__main__':
    main()
