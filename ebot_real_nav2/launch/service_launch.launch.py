
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression)


def generate_launch_description():

    
    ebot_server=Node(
        package='ebot_control',
        namespace='', 
        executable='ebot_dock_service',
        )


    rack_server = Node(
        package='ebot_control',
        namespace='',
        executable='server',
    )

    cam_script = Node(
        package='pymoveit2',
        namespace='',
        executable='cam_script.py',
    )
    
    imu_duplicate= Node(
        package='ebot_control',
        namespace='',
        executable='duplicate_imu',
     
    )


    return LaunchDescription([
        ebot_server,
        RegisterEventHandler(
            OnProcessStart(
                target_action=ebot_server,
                on_start=[
                    LogInfo(msg='service started, moving ebot'),
                    rack_server
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessStart(
                target_action=rack_server,
                on_start=[
                    LogInfo(msg='service started, moving ebot'),
                    cam_script
                ]
            )
        ),
        
        RegisterEventHandler(
            OnProcessStart(
                target_action=cam_script,
                on_start=[
                    LogInfo(msg='service started, moving ebot'),
                    imu_duplicate
                ]
            )
        ),
    ])
