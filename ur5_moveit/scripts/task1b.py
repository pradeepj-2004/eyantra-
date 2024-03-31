#!/usr/bin/env python3

from os import path
import sys
from threading import Thread
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from pymoveit2 import MoveIt2
from pymoveit2.robots import ur5

DEFAULT_EXAMPLE_MESH = path.join(
    path.dirname(path.realpath(__file__)), "assets", "rack.stl"
)

class move_pose(Node):

    def __init__(self):

        super().__init__("move_pose_publisher")
        
        self.rack_pos(0.54,0.05,-0.56,0.0,0.0,0.0,1.0,0)                    #Adding collision racks
        self.rack_pos(0.25,-0.64,-0.56,0.0, 0.0, -0.7068252, 0.7073883,1)
        self.rack_pos(0.25,0.72,-0.56,0.0, 0.0, -0.7068252, 0.7073883,2)
        for q in range(10):
            self.rack_pos(0.54,0.05,-0.56,0.0,0.0,0.0,1.0,1)                    #Adding collision racks
            self.rack_pos(0.25,-0.64,-0.56,0.0, 0.0, -0.7068252, 0.7073883,2)
            self.rack_pos(0.25,0.72,-0.56,0.0, 0.0, -0.7068252, 0.7073883,3)


        time.sleep(1)
        self.move_to_poses(0.35,0.1,0.68,0.5,0.5,0.5,0.5)                    #Moving to poses
        self.move_to_poses(-0.37, 0.12, 0.397,0.0, -0.7068252, 0.0, 0.7073883)
        self.move_to_poses(0.194,-0.43,0.701,0.7068252, 0.0, 0.0, 0.7073883)
        self.move_to_poses(-0.37, 0.12, 0.397,0.0, -0.7068252, 0.0, 0.7073883)

        exit()


    def rack_pos(self,x,y,z,rx,ry,rz,rw,t):
        if t==0:
            self.declare_parameter(
                "filepath",
                "",
            )
            self.declare_parameter(
                "action",
                "add",
            )
            self.declare_parameter("position", [x, y, z])
            self.declare_parameter("quat_xyzw", [rx, ry, rz, rw])
            
        else:
            self.declare_parameter("position", [x, y, z])
            self.declare_parameter("quat_xyzw", [rx, ry, rz, rw])
        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Get parameters
        filepath = self.get_parameter("filepath").get_parameter_value().string_value
        action = self.get_parameter("action").get_parameter_value().string_value
        position = self.get_parameter("position").get_parameter_value().double_array_value
        quat_xyzw = self.get_parameter("quat_xyzw").get_parameter_value().double_array_value

        # Use the default example mesh if invalid
        if not filepath:
            self.get_logger().info(f"Using the default example mesh file")
            filepath = DEFAULT_EXAMPLE_MESH

        # Make sure the mesh file exists
        if not path.exists(filepath):
            self.get_logger().error(f"File '{filepath}' does not exist")
            rclpy.shutdown()
            exit(1)

        # Determine ID of the collision mesh
        mesh_id = path.basename(filepath).split(".")[0]+str(t)

        if "add" == action:
            # Add collision mesh
            self.get_logger().info(
                f"Adding collision mesh '{filepath}' {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
            )
            # print(ur5.base_link_name())
            moveit2.add_collision_mesh(
                filepath=filepath, id=mesh_id, position=position, quat_xyzw=quat_xyzw, frame_id=ur5.base_link_name()
            )
        else:
            # Remove collision mesh
            self.get_logger().info(f"Removing collision mesh with ID '{mesh_id}'")
            moveit2.remove_collision_mesh(id=mesh_id)
        self.undeclare_parameter("position")
        self.undeclare_parameter("quat_xyzw")
        time.sleep(2)

        
        
    
    def move_to_poses(self,x,y,z,rx,ry,rz,rw):
        
        self.declare_parameter("position", [x, y, z])
        self.declare_parameter("quat_xyzw", [rx,ry,rz,rw])
        self.declare_parameter("cartesian", False)

        # Create callback group that allows execution of callbacks in parallel without restrictions
        callback_group = ReentrantCallbackGroup()

        # Create MoveIt 2 interface
        moveit2 = MoveIt2(
            node=self,
            joint_names=ur5.joint_names(),
            base_link_name=ur5.base_link_name(),
            end_effector_name=ur5.end_effector_name(),
            group_name=ur5.MOVE_GROUP_ARM,
            callback_group=callback_group,
        )

        # Spin the node in background thread(s)
        executor = rclpy.executors.MultiThreadedExecutor(2)
        executor.add_node(self)
        executor_thread = Thread(target=executor.spin, daemon=True, args=())
        executor_thread.start()

        # Get parameters
        position = self.get_parameter("position").get_parameter_value().double_array_value
        quat_xyzw = self.get_parameter("quat_xyzw").get_parameter_value().double_array_value
        cartesian = self.get_parameter("cartesian").get_parameter_value().bool_value

        # Move to pose
        self.get_logger().info(
            f"Moving to {{position: {list(position)}, quat_xyzw: {list(quat_xyzw)}}}"
        )
        moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        moveit2.wait_until_executed()

        self.undeclare_parameter("position")
        self.undeclare_parameter("quat_xyzw")
        self.undeclare_parameter("cartesian")


def main():

    rclpy.init(args=sys.argv)
    node = rclpy.create_node('move_pose')   #Creating node
    node.get_logger().info('Node created')
    moveit = move_pose()                        #Calling move_pose class 

    rclpy.spin(moveit)
    moveit.destroy_node()                                   # destroy node after spin ends
    rclpy.shutdown()

if __name__ == "__main__":
    main()


