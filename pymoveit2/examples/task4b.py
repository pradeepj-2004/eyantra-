#!/usr/bin/env python3
'''
# Team ID:          < 2083 >
# Theme:            < Cosmo Logistic >
# Author List:      < Pradeep,Mohit,Santosh,Parth >
# Filename:         < test_tf >
# Functions:        < __init__,send_goal,move_to_poses,rack_pos,servo_circular_motion,attach_box,send_request,detach_box,callback,check_side,joint_pose,wait_for_callback_result >
# Global variables: <l1>
'''

from rclpy.node import Node
from threading import Thread
import numpy as np
import tf_transformations as tf
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from geometry_msgs.msg import TwistStamped,TransformStamped
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion
from std_msgs import *
import math,time
from linkattacher_msgs.srv import AttachLink,DetachLink
from std_srvs.srv import Trigger
from std_msgs.msg import Int8
import tf2_ros
from example_interfaces.srv import AddTwoInts

need=[0]
box_num=[0]
class move_pose(Node):

    def __init__(self):
        super().__init__('move_pose_node')
        
        # self.subscription=self.create_subscription(TFMessage,'/tf', self.callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.start_service = self.create_client(srv_type=Trigger,srv_name="/servo_node/start_servo")
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_control = self.create_client(DetachLink,'/GripperMagnetOFF')
        
        self.tf_buffer = tf2_ros.buffer.Buffer()                                        # buffer time used for listening transforms
        self.listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.05, self.callback) 
        self.servo_check=self.create_subscription(Int8,'/servo_node/status',self.servo_status,10)
        self.status=0
                
        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(node=self,joint_names=ur5.joint_names(),base_link_name=ur5.base_link_name(),end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
        
        #service part
        self.rack_sub=self.create_client(AddTwoInts,'arm_starter')
    
    def send_goal(self,msg):
        goal_msg = AddTwoInts.Request()
        goal_msg.a = msg
        return self.rack_sub.call_async(goal_msg)
    
    def servo_status(self,num):
        self.status=num
        
    def callback(self):
        list_1=[]
        # print('callback')
        if need[0]==1:
            try:
                # Look up the transform between 'base_link' and 'obj_<marker_id>'
                transform = self.tf_buffer.lookup_transform('base_link', "CL_2083_base_"+str(box_num[0]), rclpy.time.Time())
                tf_msg = TransformStamped()
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation=transform.transform.rotation
                list_1=[tf_msg.transform.translation.x,tf_msg.transform.translation.y,tf_msg.transform.translation.z,tf_msg.transform.rotation.x,tf_msg.transform.rotation.y,tf_msg.transform.rotation.z,tf_msg.transform.rotation.w]
            
            except TransformException as e:
                # print("wait for","CL_2083_base_"+str(box_num[0]))
                return
            if list_1!=[]:
                self.future.set_result(list_1)
        
        elif need[0]==2:
            try:
                # Look up the transform between 'base_link' and 'obj_<marker_id>'
                transform = self.tf_buffer.lookup_transform('base_link', "tool0", rclpy.time.Time())
                tf_msg = TransformStamped()
                tf_msg.transform.translation = transform.transform.translation
                tf_msg.transform.rotation=transform.transform.rotation
                list_1=[tf_msg.transform.translation.x,tf_msg.transform.translation.y,tf_msg.transform.translation.z,tf_msg.transform.rotation.x,tf_msg.transform.rotation.y,tf_msg.transform.rotation.z,tf_msg.transform.rotation.w]
            
            except TransformException as e:
                # print("wait for tool0")
                return 
            if list_1!=[]:
                self.future.set_result(list_1)
    
    def joint_pose(self,j1,j2,j3,j4,j5,j6):
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()
        self.moveit2.move_to_configuration([j1,j2,j3,j4,j5,j6])
        self.moveit2.wait_until_executed()
    
    def enable(self):
        return self.start_service.call_async(Trigger.Request())
    
    
    def servo_circular_motion(self,speed_x,speed_y,speed_z,l):
        i=0
        # executor = rclpy.executors.MultiThreadedExecutor(2)
        # executor.add_node(self)
        # executor_thread = Thread(target=executor.spin, daemon=True, args=())
        # executor_thread.start()
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
            
            self.twist_pub.publish(self.twist_msg)
            i=i+1
            time.sleep(0.01)
        
    def attach_box(self,box_name):
        while not self.gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('EEF service not available, waiting again...')
        req = AttachLink.Request()
        req.model1_name = box_name   
        req.link1_name  = 'link'       
        req.model2_name = 'ur5'       
        req.link2_name  = 'wrist_3_link'  
        self.gripper_control.call_async(req)
        print(box_name,"attached")

    def detach_box(self,box_name):
        while not self.detach_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('EEF service not available, waiting again...')

        req =DetachLink.Request()
        req.model1_name = box_name   
        req.link1_name  = 'link'       
        req.model2_name = 'ur5'       
        req.link2_name  = 'wrist_3_link'  
        self.detach_control.call_async(req)
        print(box_name,"de-attached")
    
    
    def wait_for_callback_result(self):
        self.future=rclpy.Future()
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def transform(self,tool0,obj):
        tool0_to_base = tf.translation_matrix([tool0[0], tool0[1], tool0[2]]) @ tf.quaternion_matrix([tool0[3], tool0[4], tool0[5], tool0[6]])
        obj1_to_base = tf.translation_matrix([obj[0],obj[1], obj[2]]) @ tf.quaternion_matrix([obj[3], obj[4], obj[5], obj[6]])
        tool0_to_obj1 = np.linalg.inv(obj1_to_base) @ tool0_to_base

        translation_tool0_to_obj1 = tf.translation_from_matrix(tool0_to_obj1)
        rotation_tool0_to_obj1 = tf.quaternion_from_matrix(tool0_to_obj1)
        return translation_tool0_to_obj1,rotation_tool0_to_obj1

def main(args=None):
    rclpy.init(args=args)
    
    main_object = move_pose()  #creating Object for class move_pose
    
    while not main_object.start_service.wait_for_service(timeout_sec=1.0):
            main_object.get_logger().warn('Servo service not available, waiting again...')
    main_object.enable()
    main_object.joint_pose( 0.0,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066) 

    for j in range(3): 
        
        while True:
            while not main_object.rack_sub.wait_for_service(timeout_sec=1.0):
                main_object.get_logger().warn('Rack service not available, waiting again...')
            future = main_object.send_goal(0)                                                #Sending request for service which will respond with tool 0 tf_value
            rclpy.spin_until_future_complete(main_object, future)
            
            if(future.result().sum==j+1):
                break
            time.sleep(1) 
    
        if j==0:
            box_num[0]=3
        elif j==1:
            box_num[0]=1
        elif j==2:
            box_num[0]=2

            
        need[0]=2
        tool_pose=main_object.wait_for_callback_result()

        need[0]=1
        obj_pose=main_object.wait_for_callback_result()

        transform_trans,transform_rot=main_object.transform(obj_pose,tool_pose)
        transform_rot=euler_from_quaternion(transform_rot)
        l1=[transform_trans[0],transform_trans[1],transform_trans[2],transform_rot[0],transform_rot[1],transform_rot[2]]
        if abs(l1[3])>3.1:
            if l1[4]>0:
                l1[4]=3.14-l1[4]
            else:
                l1[4]=-3.14-(l1[4])
                
        main_object.joint_pose( float(l1[4]),-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066)

        need[0]=2
        tool_pose=main_object.wait_for_callback_result()

        transform_trans,transform_rot=main_object.transform(obj_pose,tool_pose)
        transform_rot=euler_from_quaternion(transform_rot)
        l1=[transform_trans[0],transform_trans[1],transform_trans[2],transform_rot[0],transform_rot[1],transform_rot[2]]

        m1,m2,m3,r1,r2,r3=0,0,0,0.0,0.0,0.0
        
        transform_trans,transform_rot=main_object.transform(obj_pose,tool_pose)
        sign=0
        if transform_trans[0]>0:
            sign=1
        else:
            sign=-1
        
        while True:
            need[0]=2            
            tool_pose=main_object.wait_for_callback_result()

            transform_trans,transform_rot=main_object.transform(obj_pose,tool_pose)

            c=transform_trans[1]
            a=transform_trans[2]
            b=transform_trans[0]
            if sign==1 :
                k=1
            else:
                b=-b
                k=-1

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
            if (m1==1 and m3==1 and m2==1):
                break
            
        while True:
            future=main_object.wait_for_callback_result()
            if (math.sqrt(obj_pose[0]**2+obj_pose[1]**2+obj_pose[2]**2)>math.sqrt((future[0])**2+(future[1])**2+(future[2])**2)):    #Servoing till it reaches the box
                main_object.servo_circular_motion(0.0,0.0,0.1,1)
                if main_object.status.data!=0:
                    print('move to next because signularity')
                    break
                
            else:
                print(tool_pose,future)
                print("Servoing done for picking!!")
                break 
            
        main_object.attach_box('box'+str(box_num[0])) 

        trav=[0,0,0]
        while True:
                                                                      #Sending request for service which will respond with tool 0 tf_value
            future=main_object.wait_for_callback_result()

            if (tool_pose[2]+0.03>=future[2]):    #Servoing till it reaches the box height
                main_object.servo_circular_motion(0.0,0.1,0.0,1)
                trav[0]=future[0]
                trav[1]=future[1]
                trav[2]=future[2]
                if main_object.status.data!=0:
                    print('move to next because signularity')
                    break
            else:
                print("Servoing done for top!!")
                break
            
        while True:
            future=main_object.wait_for_callback_result()
            if (math.sqrt(((trav[0]-future[0])**2)+((trav[1]-future[1])**2)+(trav[1]-future[1])**2)<0.23):   #Servoing backwards to avoid collision of box with rack 
                if main_object.status.data!=0:
                    print('move to next because signularity')
                    break 
                main_object.servo_circular_motion(0.0,0.0,-0.1,1)
                                   
            else:
                print('back servo done')
                break

        time.sleep(1)

        main_object.joint_pose( 0.0, -1.57, 0.0, -3.14, -1.57, 0.0)                              #Moving to intermediate drop location

        if j==0:
            main_object.joint_pose( -0.0, -2.4436, -0.8028, -3.01942, -1.57,-3.14)                  #Moving to  drop location using joint pose
        elif j==1:
            main_object.joint_pose( 0.0, -1.9547687622336491, -1.5533430342749532, -2.827433388230814, -1.5707963267948966, 3.141592653589793)
        else :
            main_object.joint_pose( 0.0, -2.0943951023931953, -1.0297442586766545, -3.07177948351002, -1.5707963267948966, 3.141592653589793)   

        main_object.detach_box('box'+str(box_num[0]))
        main_object.joint_pose( 0.0,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066) # Moving back to home location
    
    # node.destroy_node()                                   # destroy node after spin ends
    rclpy.shutdown()
    exit()
    
if __name__ == '__main__':
    main()
        


