#!/usr/bin/env python3
'''
# Team ID:          < 2083 >
# Theme:            < Cosmo Logistic >
# Author List:      < Pradeep,Mohit,Santosh,Parth >
# Filename:         < test_tf >
# Functions:        < __init__,send_goal,move_to_poses,rack_pos,servo_circular_motion,attach_box,send_request,detach_box,callback,check_side,joint_pose,wait_for_callback_result >
# Global variables: <l1>
'''

import rclpy
from rclpy.node import Node
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
import tf2_ros
from tf2_ros import TFMessage
from geometry_msgs.msg import TwistStamped,TransformStamped
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from std_msgs import *
import math
from std_srvs.srv import Trigger
from ur_msgs.srv import SetIO
from controller_manager_msgs.srv import SwitchController


need=[0]
box_num=[0]

#check how much needed to servo back

class move_pose(Node):

    def __init__(self):
        super().__init__('move_pose_publisher')
        
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.start_service = self.create_client(srv_type=Trigger,srv_name="/servo_node/start_servo")

        
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.01, self.callback) 
                        
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(node=self,joint_names=ur5.joint_names(),base_link_name=ur5.base_link_name(),end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

    def enable(self):
        return self.start_service.call_async(Trigger.Request())
    
    def gripper_call(self, state):
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)   #Check once whether 0 or 1
        gripper_control.call_async(req)
        print("Gripper call activated")
        return state
    
    def traj_switch(self,val):
        switchParam = SwitchController.Request()
        if val==1:
            # for normal use of moveit
            switchParam.activate_controllers = ["scaled_joint_trajectory_controller"] 
            switchParam.deactivate_controllers = ["forward_position_controller"] 
            res='moveit'
        else:
            # for servoing
            switchParam.deactivate_controllers = ["scaled_joint_trajectory_controller"] 
            switchParam.activate_controllers = ["forward_position_controller"] 
            res='servo' 
        switchParam.strictness = 2
        switchParam.start_asap = False

        while not self.__contolMSwitch.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"Service control Manager is not yet available...")
        self.__contolMSwitch.call_async(switchParam)
        print("[CM]: Switching Complete"+res) 
    
    def servo_circular_motion(self,speed_x,speed_y,speed_z,l):
        i=0
        while i<l:
            self.twist_msg = TwistStamped()
            self.twist_msg.header.frame_id=ur5.end_effector_name()
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.twist.linear.x = speed_x*2.5
            self.twist_msg.twist.linear.y = speed_y*2.5
            self.twist_msg.twist.linear.z = speed_z*2.5
            self.twist_msg.twist.angular.x = 0.0
            self.twist_msg.twist.angular.y = 0.0
            self.twist_msg.twist.angular.z = 0.0
            executor = rclpy.executors.MultiThreadedExecutor(2)
            executor.add_node(self)
            executor_thread = Thread(target=executor.spin, daemon=True, args=())
            executor_thread.start()
            self.twist_pub.publish(self.twist_msg)
            i=i+1
            time.sleep(0.01)
        
    
    def callback(self):
        list_1=[]
        if need[0]==1:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', "CL_2083_base_"+str(box_num[0]), rclpy.time.Time())
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'base_link'
                tf_msg.child_frame_id = 'new'
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation=transform.transform.rotation
                list_1=[tf_msg.transform.translation.x,tf_msg.transform.translation.y,tf_msg.transform.translation.z,tf_msg.transform.rotation.x,tf_msg.transform.rotation.y,tf_msg.transform.rotation.z,tf_msg.transform.rotation.w]
            
            except TransformException as e:
                print("wait for","CL_2083_base_"+str(box_num[0]))

            if list_1!=[]:
                euler1=euler_from_quaternion([list_1[3],list_1[4],list_1[5],list_1[6]])
                s1=self.check_side(euler1[0],euler1[1],euler1[2])
                list_1.append(s1)
                self.future.set_result(list_1)
        
        elif need[0]==2:
            try:
                # Look up the transform between 'base_link' and 'obj_<marker_id>'
                transform = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                tf_msg = TransformStamped()
                tf_msg.header.stamp = self.get_clock().now().to_msg()
                tf_msg.header.frame_id = 'base_link'
                tf_msg.child_frame_id = 'new'
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation=transform.transform.rotation
                list_1=[tf_msg.transform.translation.x,tf_msg.transform.translation.y,tf_msg.transform.translation.z,tf_msg.transform.rotation.x,tf_msg.transform.rotation.y,tf_msg.transform.rotation.z,tf_msg.transform.rotation.w]
            
            except TransformException as e:
                print("wait for tool0")
                
            if list_1!=[]:
                self.future.set_result(list_1)
    
    def check_side(self,a,b,c):
        if (int(a)==1 and int(b)==0 and int(c)==1):
            return "center"
        elif (int(a)==1 and int(b)==0 and int(c)==-3):
            return 'left'
        else:
            return 'right'
        
    def joint_pose(self,j1,j2,j3,j4,j5,j6):
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.moveit2.move_to_configuration([j1,j2,j3,j4,j5,j6])
        self.moveit2.wait_until_executed()

    def wait_for_callback_result(self):
        self.future=rclpy.Future()
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
                
def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('move_pose')
    node.get_logger().info('Node created')
    
    main_object = move_pose()  #creating Object for class move_pose
    while not main_object.start_service.wait_for_service(timeout_sec=1.0):
            main_object.get_logger().warn('Servo service not available, waiting again...')
    main_object.enable()
    
    for j in range(3):  
    
        if j==0:
            box_num[0]=3
        elif j==1:
            box_num[0]=1
        elif j==2:
            box_num[0]=2

        need[0]=1
        result=[]
        result=main_object.wait_for_callback_result()
        print(result[7])
        
        need[0]=0
        main_object.traj_switch(1)
        
        # main_object.joint_pose( 0, -2.398, 2.43, -3.15, -1.58, 3.15)
        
        if result[7]=='left':
            main_object.joint_pose(1.57,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066)
        elif result[7]=='right':
            main_object.joint_pose(-1.57,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066)
        
        m1,m2,m3,r1,r2,r3=0,0,0,0.0,0.0,0.0

        main_object.traj_switch(0)
        
        while True:
            need[0]=2
            
            future=main_object.wait_for_callback_result()
       
            c=result[2]-future[2]
            if result[7]=='center':
                a=result[0]-future[0]
                if result[1]>0.1:
                    b=result[1]-future[1]
                    k=1
                else:
                    b=-(result[1]-future[1])
                    k=-1
                
            elif result[7]=='right':
                if (result[0]>0.1):
                    b=result[0]-future[0]
                    k=1
                else:
                    b=-(result[0]-future[0])
                    k=-1
                a=-(result[1]-future[1])
                

            elif result[7]=='left':
                if (result[0]>-0.1):
                    b=result[0]-future[0]
                    k=-1
                else:
                    b=-(result[0]-future[0])
                    k=1
                a=result[1]-future[1]
                
            if (a-0.1>0.01):
                r3=0.1
            else:
    
                r3=0.0
                m1=1

            if (b>0.01):
                r1=0.1*k
            else:
                m2=1
            
            if (c>0.01):
                r2=0.1
            else:
                m3=1          
            
            main_object.servo_circular_motion(r1,r2,r3,1) 
            r1,r2,r3=0.0,0.0,0.0

            if (m1==1 and m2==1 and m3==1):
                print(a,b,c)
                print('servoing done to go near box')
                break
        
        while True:

            future=main_object.wait_for_callback_result()
            
            if (math.sqrt(result[0]**2+result[1]**2+result[2]**2)>math.sqrt((future[0])**2+(future[1])**2+(future[2])**2)):    #Servoing till it reaches the box
                main_object.servo_circular_motion(0.0,0.0,0.1,1)
            else:
                print("Servoing done for picking!!")
                break  
        
        main_object.gripper_call(True)                                                           #Service call for attaching box

        trav=[0,0,0]
        while True:
            
            future=main_object.wait_for_callback_result()
            if (result[2]+0.03>=future[2]):    #Servoing till it reaches the box height
                main_object.servo_circular_motion(0.0,0.1,0.0,1)
                trav[0]=future[0]
                trav[1]=future[1]
                trav[2]=future[2]
            else:
                print("Servoing done for top!!")
                break
        
        while True:
            
            future=main_object.wait_for_callback_result()
            if (abs(math.sqrt(trav[0]**2+trav[1]**2+trav[2]**2)-math.sqrt((future[0])**2+(future[1])**2+(future[2])**2))<0.3):    #Servoing till it reaches the box height
                main_object.servo_circular_motion(0.0,0.0,-0.1,1)                    #Servoing backwards to avoid collision of box with rack
            else:
                break
        
        main_object.traj_switch(1)
        
        main_object.joint_pose( 0.0, -1.57, 0.0, -3.14, -1.57, 0.0)                              #Moving to intermediate drop location
        
        main_object.joint_pose( -0.027, -1.803, -1.3658, -3.039, -1.52, 3.15)       #Moving to  drop location using joint pose
        main_object.gripper_call(False)                                                         #Sending request for deattaching box
        main_object.joint_pose( 0, -2.398, 2.43, -3.15, -1.58, 3.15)

       
    node.destroy_node()                                   # destroy node after spin ends
    rclpy.shutdown()
    exit()
    
if __name__ == '__main__':
    main()
