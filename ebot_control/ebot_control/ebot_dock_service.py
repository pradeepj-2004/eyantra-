#!/usr/bin/env python3



# Import necessary ROS2 packages and message types
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Float32MultiArray
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from ebot_docking.srv import DockSw  # Import custom service message
import math

'''
#Changable variable
kp_linear
kp_angular
distance
abs(angle_diff)
'''

# Define a class for your ROS2 node
class MyRobotDockingController(Node):

    def __init__(self):
        # Initialize the ROS2 node with a unique name
        super().__init__('my_robot_docking_controller')
        self.get_logger().info("Node started")
        # Create a callback group for managing callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        #orientation subscriber
        self.imu_sub = self.create_subscription(Float32, 'orientation', self.orientation_callback, 10)

        #Ultrasonic callback
        self.ultra_sub = self.create_subscription(Float32MultiArray, 'ultrasonic_sensor_std_float', self.ultra_callback, 10)

        # Create a ROS2 service for controlling docking behavior, can add another custom service message
        self.dock_control_srv = self.create_service(DockSw, 'my_service_dock', self.dock_control_callback, callback_group=self.callback_group)

        # Create a publisher for sending velocity commands to the robot
        self.vel_publisher = self.create_publisher(Twist, "/cmd_vel", 10)


        # Initialize all  flags and parameters here
        self.is_docking = False
        self.linear_dock= False
        self.angular_dock = False

        # Initialize a timer for the main control loop
        self.controller_timer = self.create_timer(0.1, self.controller_loop)

    # Callback function for ultrasonic sensor
    def ultra_callback(self,msg):
        self.usrleft_value= (msg.data[4])/100
        self.usrright_value = (msg.data[5])/100
    
    #callback of imu
    def orientation_callback(self, msg):
        self.yaw_value = msg.data

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
            kp_linear=0.3
            velocity=kp_linear*distance
            kp_angular=2.0
                         
            if (self.angular_dock==True and self.linear_dock==True):
                
                if distance <= 0.21 and self.turn==False:
                    self.send_velocity_command(0.0,0.0)
                    self.is_docking=False
                    print("linear docking done")
                elif (self.turn==False):
                    print('linear_vel:',velocity,"distance:",distance)
                    self.send_velocity_command(-velocity,0.0)
                
                if (self.turn==True):
                    angle_rad=self.dock_angle-self.yaw_value

                    if (abs(angle_rad)>math.pi):
                        if (self.yaw_value>math.pi):
                            angle_rad=2*math.pi-self.yaw_value+self.dock_angle
                            omega=-kp_angular*(2*math.pi-self.yaw_value+self.dock_angle)
                        elif(self.yaw_value<math.pi):
                            angle_rad=2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle
                            omega=-kp_angular*(2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle)

                    else:
                        omega=-(kp_angular*angle_rad)

                    angle_diff=angle_rad*180/3.14

                    if (abs(angle_diff) >=1.5):
                        self.send_velocity_command(0.0,omega)
                        print('ang_speed:',omega,' angle_diff:',angle_diff)
                    else:
                        print("angular docking done") 
                        self.send_velocity_command(0.0,0.0)
                        self.turn=False
         
            elif (self.angular_dock==True and self.linear_dock==False):
                angle_rad=self.dock_angle-self.yaw_value
  
                if (abs(angle_rad)>math.pi):
                    if (self.yaw_value>math.pi):
                        angle_rad=2*math.pi-self.yaw_value+self.dock_angle
                        omega=-kp_angular*(2*math.pi-self.yaw_value+self.dock_angle)
                    elif(self.yaw_value<math.pi):
                        angle_rad=2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle
                        omega=-kp_angular*(2*math.pi-(self.yaw_value+2*math.pi)+(2*math.pi)-self.dock_angle)
                    
                else:
                    omega=-(kp_angular*angle_rad)
                
                angle_diff=angle_rad*180/3.14

                if (abs(angle_diff) >=1.5):
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

if __name__== '__main__':
    main()
