#! /usr/bin/env python3

import time # Time library
 
from geometry_msgs.msg import PoseStamped # Pose with ref frame and timestamp
from rclpy.duration import Duration # Handles time for ROS 2
import rclpy # Python client library for ROS 2

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

 
'''
Follow waypoints using the ROS 2 Navigation Stack (Nav2)
'''
def main():
 
  # Start the ROS 2 Python Client Library
  rclpy.init()
 
  # Launch the ROS 2 Navigation Stack
  navigator = BasicNavigator()
 
  # Set the robot's initial pose if necessary
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
 
 
  goal_poses = []
   
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 0.3
  goal_pose.pose.position.y = 4.5
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.9999997
  goal_pose.pose.orientation.w = 0.0007963
  goal_poses.append(goal_pose)
   

 
  nav_start = navigator.get_clock().now()
  navigator.followWaypoints(goal_poses)
 
  i = 0
  while not navigator.isTaskComplete():

    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 5 == 0:
      print('Executing current waypoint: ' +
            str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
      now = navigator.get_clock().now()
 

      if now - nav_start > Duration(seconds=100000000.0):
        navigator.cancelNav()
 

  result = navigator.getResult()
  if result == TaskResult.SUCCEEDED:
    print('Goal succeeded!')
  elif result == TaskResult.CANCELED:
    print('Goal was canceled!')
  elif result == TaskResult.FAILED:
    print('Goal failed!')
  else:
    print('Goal has an invalid return status!')
 
  navigator.lifecycleShutdown()
 
  exit(0)
 
if __name__ == '__main__':
  main()


# #!/usr/bin/env python3

# import time
# from geometry_msgs.msg import PoseStamped
# from rclpy.duration import Duration
# import rclpy
# from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

# class NavigationController:
#     def __init__(self):
#         rclpy.init()
#         self.navigator = BasicNavigator()
#         self.setup_initial_pose()
    
#     def setup_initial_pose(self):
#         initial_pose = PoseStamped()
#         initial_pose.header.frame_id = 'map'
#         initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
#         initial_pose.pose.position.x = 0.0
#         initial_pose.pose.position.y = 0.0
#         initial_pose.pose.position.z = 0.05
#         initial_pose.pose.orientation.x = 0.0
#         initial_pose.pose.orientation.y = 0.0
#         initial_pose.pose.orientation.z = 0.0
#         initial_pose.pose.orientation.w = 1.0
#         self.navigator.setInitialPose(initial_pose)
    
#     def follow_waypoints(self, goal_poses):
#         self.navigator.waitUntilNav2Active()
#         self.navigator.followWaypoints(goal_poses)
        
#     def is_task_complete(self):
#         return self.navigator.isTaskComplete()
    
#     def execute_navigation(self):
#         i = 0
#         while not self.is_task_complete():
#             i = i + 1
#             feedback = self.navigator.getFeedback()
#             if feedback and i % 5 == 0:
#                 print('Executing current waypoint: ' +
#                       str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))
#                 now = self.navigator.get_clock().now()

#                 if now - nav_start > Duration(seconds=100000000.0):
#                     self.navigator.cancelNav()
        
#         result = self.navigator.getResult()
#         return result

#     def shutdown(self):
#         self.navigator.lifecycleShutdown()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     navigation_controller = NavigationController()
    
#     goal_poses = []
#     goal_pose = PoseStamped()
#     goal_pose.header.frame_id = 'map'
#     goal_pose.header.stamp = navigation_controller.navigator.get_clock().now().to_msg()
#     goal_pose.pose.position.x = 0.3
#     goal_pose.pose.position.y = 4.5
#     goal_pose.pose.position.z = 0.0
#     goal_pose.pose.orientation.x = 0.0
#     goal_pose.pose.orientation.y = 0.0
#     goal_pose.pose.orientation.z = 0.9999997
#     goal_pose.pose.orientation.w = 0.0007963
#     goal_poses.append(goal_pose)
    
#     navigation_controller.follow_waypoints(goal_poses)
#     result = navigation_controller.execute_navigation()
    
#     if result == TaskResult.SUCCEEDED:
#         print('Goal succeeded!')
#     elif result == TaskResult.CANCELED:
#         print('Goal was canceled!')
#     elif result == TaskResult.FAILED:
#         print('Goal failed!')
#     else:
#         print('Goal has an invalid return status!')
    
#     navigation_controller.shutdown()
#     exit(0)
