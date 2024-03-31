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
from ur5_moveit.srv import Tfvalue

import sys
from threading import Thread
import time
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5
from tf2_ros import TFMessage
from geometry_msgs.msg import TwistStamped
from tf_transformations import euler_from_quaternion
from std_msgs import *
import math
from linkattacher_msgs.srv import AttachLink,DetachLink
from ur5_moveit.srv import Ur5 
from std_srvs.srv import Trigger

box_num=[0]
class move_pose(Node):

    def __init__(self):
        super().__init__('move_pose_publisher')
        self.tf_value = self.create_client(Tfvalue, 'my_service_pradeep')
        self.subscription=self.create_subscription(TFMessage,'/tf', self.callback, 10)
        self.twist_pub = self.create_publisher(TwistStamped, "/servo_node/delta_twist_cmds", 10)
        self.start_service = self.create_client(srv_type=Trigger,srv_name="/servo_node/start_servo")
        self.gripper_control = self.create_client(AttachLink, '/GripperMagnetON')
        self.detach_control = self.create_client(DetachLink,'/GripperMagnetOFF')

        callback_group = ReentrantCallbackGroup()
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )
    def enable(self):
        return self.start_service.call_async(Trigger.Request())

    def send_goal(self,msg):
        goal_msg = Tfvalue.Request()
        goal_msg.message = msg
        return self.tf_value.call_async(goal_msg)
   
    def servo_circular_motion(self,speed_x,speed_y,speed_z,l):
        i=0
        while i<l:
            self.twist_msg = TwistStamped()
            self.twist_msg.header.frame_id=ur5.base_link_name()
            self.twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.twist_msg.twist.linear.x = speed_x
            self.twist_msg.twist.linear.y = speed_y
            self.twist_msg.twist.linear.z = speed_z
            self.twist_msg.twist.angular.x = 0.0
            self.twist_msg.twist.angular.y = 0.0
            self.twist_msg.twist.angular.z = 0.0
            self.twist_pub.publish(self.twist_msg)
            i=i+1
            time.sleep(0.1)
        
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
    
    def callback(self,msg):
        l1=[]        
        frame_ids = set()
        for transform in msg.transforms:
            frame_ids.add(transform.child_frame_id)
            frame_ids.add(transform.header.frame_id)
            
        for frame_id in frame_ids:
            if frame_id==("obj_"+str(box_num[0])):
                self.position = transform.transform.translation
                self.rot=transform.transform.rotation

                if (l1==[] and self.position!=None):
                    l1.extend([self.position.x,self.position.y,self.position.z,self.rot.x,self.rot.y,self.rot.z,self.rot.w])
                    y1=box_num[0]
                    break
                else:
                    continue

        if (l1!=[]):  
            list_1=[l1[0],l1[1],l1[2],l1[3],l1[4],l1[5],l1[6]]
            list_euler1=[l1[3],l1[4],l1[5],l1[6]]
            euler1=euler_from_quaternion(list_euler1)
            s1=self.check_side(euler1[0],euler1[1],euler1[2])
            l1_x,l1_y=0.0,0.0
            if s1 == 'center':
                    l1_x=0.1
                    l1_y=0.0
            elif s1 == 'right':
                    l1_x=0.0
                    l1_y=-0.1
            else:
                    l1_x=0.0
                    l1_y=0.1
            list_1.extend([l1_x,l1_y,y1,s1])
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
            box_num[0]=49
        elif j==1:
            box_num[0]=3
        elif j==2:
            box_num[0]=1

        result=[]
        result = main_object.wait_for_callback_result()                                 #This will trigger a function in class which waits till result of aruco marker comes and return values
        print(result[10])
        if result[10]=='left':
            main_object.joint_pose(1.57,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066)
        elif result[10]=='right':
            main_object.joint_pose(-1.57,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066)
        
        m1,m2,m3,r1,r2,r3=0,0,0,0.0,0.0,0.0

        while True:
            while not main_object.tf_value.wait_for_service(timeout_sec=1.0):
                main_object.get_logger().warn('EEF service not available, waiting again...')
            
            future = main_object.send_goal('tool0')                                                #Sending request for service which will respond with tool 0 tf_value
            rclpy.spin_until_future_complete(main_object, future)

            
            if (result[2]>-future.result().ty):    #Servoing till it reaches the box height
                r3=0.1
            else:
                m1=1


            if (result[10]=='right'):
                if(result[0]>-future.result().tx):
                    r1=0.1
                else:
                    m2=1

                if(result[1]+0.1<future.result().tz):
                    r1=r1+result[7]
                    r2=r2+result[8]
                else:
                    m3=1


            if (result[10]=='left'):
                if(result[0]>future.result().tx):
                    r1=0.1
                else:
                    m2=1

                if(result[1]-0.1>-future.result().tz):
                    r1=r1+result[7]
                    r2=r2+result[8]
                else:
                    m3=1


            if (result[10]=='center'):
                if(result[0]-0.1>-future.result().tz):
                    r1=r1+result[7]
                    r2=r2+result[8]
                else:
                    m3=1

                if(result[1]>0.1):
                    k=1
                else:
                    k=-1
                if(k*result[1]>-future.result().tx*k):
                    r2=0.1*k     
                else:
                    m2=1             

            main_object.servo_circular_motion(r1,r2,r3,1) 

            r1,r2,r3=0.0,0.0,0.0

            if (m1==1 and m2==1 and m3==1):
                print("Servoing done to go near box")
                break

        while True:
            while not main_object.tf_value.wait_for_service(timeout_sec=1.0):
                main_object.get_logger().warn('EEF service not available, waiting again...')
            
            future = main_object.send_goal('tool0')                                                #Sending request for service which will respond with tool 0 tf_value
            rclpy.spin_until_future_complete(main_object, future)
            if (math.sqrt(result[0]**2+result[1]**2+result[2]**2)>math.sqrt((future.result().tx)**2+(future.result().ty)**2+(future.result().tz)**2)):    #Servoing till it reaches the box
                main_object.servo_circular_motion(result[7]*0.5,result[8]*0.5,0.0,1)
            else:
                print("Servoing done for picking!!")
                break                                                                           #Breaking after servoing done

        main_object.attach_box('box'+str(result[9]))                                                          #Service call for attaching box

        trav=[0,0,0]
        while True:
            while not main_object.tf_value.wait_for_service(timeout_sec=1.0):
                main_object.get_logger().warn('EEF service not available, waiting again...')
            
            future = main_object.send_goal('tool0')                                                #Sending request for service which will respond with tool 0 tf_value
            rclpy.spin_until_future_complete(main_object, future)
            if (result[2]+0.03>=-future.result().ty):    #Servoing till it reaches the box height
                main_object.servo_circular_motion(0.0,0.0,0.1,1)
                trav[0]=future.result().ty
                trav[1]=future.result().tx
                trav[2]=future.result().tz
                
            else:
                print("Servoing done for top!!")
                break
        
        while True:
            while not main_object.tf_value.wait_for_service(timeout_sec=1.0):
                main_object.get_logger().warn('EEF service not available, waiting again...')
            
            future = main_object.send_goal('hi')                                                #Sending request for service which will respond with tool 0 tf_value
            rclpy.spin_until_future_complete(main_object, future)
            if (abs(math.sqrt(trav[0]**2+trav[1]**2+trav[2]**2)-math.sqrt((future.result().tx)**2+(future.result().ty)**2+(future.result().tz)**2))<0.06):    #Servoing till it reaches the box height
                main_object.servo_circular_motion(-result[7],-result[8],0.0,1)                    #Servoing backwards to avoid collision of box with rack

            else:
                print(abs(math.sqrt(trav[0]**2+trav[1]**2+trav[2]**2)-math.sqrt((future.result().tx)**2+(future.result().ty)**2+(future.result().tz)**2)))
                print("Servoing done for backwards!!")
                break
        
        main_object.joint_pose( 0.0, -1.57, 0.0, -3.14, -1.57, 0.0)                              #Moving to intermediate drop location
        # main_object.servo_circular_motion(0.0,0.0,0.1,100)
        if(j==0):
            main_object.joint_pose( -0.0, -2.4436, -0.8028, -3.01942, -1.57,-3.14)                  #Moving to  drop location using joint pose
        elif(j==1):
            main_object.joint_pose( -0.0, -2.4436, -0.8028, -3.01942, -1.57,-3.14)                  #Moving to  drop location using joint pose
        elif(j==2):
            main_object.joint_pose( -0.0, -2.4436, -0.8028, -3.01942, -1.57,-3.14)                  #Moving to  drop location using joint pose

        main_object.detach_box('box'+str(result[9]))                                                          #Sending request for deattaching box
        main_object.joint_pose( 0.0,-2.390102401972993,2.399966716334568, -3.150040830693781, -1.580073468092209, 3.1500244180924066) # Moving back to home location
        result=[]

    node.destroy_node()                                   # destroy node after spin ends
    rclpy.shutdown()
    exit()
if __name__ == '__main__':
    main()