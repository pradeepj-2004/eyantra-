#!/usr/bin/env python3


# Ensure that the robot is at the home position before starting the script.

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from usb_relay.srv import RelaySw
from ur_msgs.srv import SetIO

class reset_object(Node):
    def __init__(self):
        super().__init__('reset_node')

        ####### Functions to be called only once #######         
        # self.reset_imu()                                    # Reset IMU data
        # self.reset_odom()                                   # Reset Odom

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

    def gripper_call(self, state):
        gripper_control = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not gripper_control.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('EEF Tool service not available, waiting again...')
        req         = SetIO.Request()
        req.fun     = 1
        req.pin     = 16
        req.state   = float(state)   #Check once whether 0 or 1
        
        self.arm_gripper_resp=gripper_control.call_async(req)
        rclpy.spin_until_future_complete(self, self.arm_gripper_resp)
        # if(self.arm_gripper_resp.result().success== True):
        #     self.get_logger().info(self.arm_gripper_resp.result().message)
        # else:
        #     self.get_logger().warn(self.arm_gripper_resp.result().message)

        
        print("Gripper call done")
        return state
   

def main(args=None):
    rclpy.init(args=args)
    node = reset_object()
    
    node.reset_odom()
    node.reset_imu()
    node.switch_eletromagent(False)
    node.gripper_call(False)
    
if __name__ == '__main__':
    main()