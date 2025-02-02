import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ariac_moveit_config.parameters import generate_parameters
from launch.actions import TimerAction 

def launch_setup(context, *args, **kwargs):
    demo_cpp = Node(
        package="final_group1",
        executable="moveit_demo_cpp_python_exe",
        output="screen",
        parameters= generate_parameters()
    )

    start_rviz = LaunchConfiguration("rviz")

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("final_group1"), "rviz", "moveit_demo.rviz"]
    )

    
    delay_timer2 = TimerAction(
        period=5.0,  # 2 seconds delay
        actions=[
            Node(
                package="final_group1",
                executable="competition_node.py",
                emulate_tty=True,
                #output="screen"
            )
        ]
    )
    delay_timer1 = TimerAction(
        period=3.0,  # 2 seconds delay
        actions=[
            Node(
                package="final_group1",
                executable="kitting_node.py",
                emulate_tty=True,
                output="log",
                parameters=[ {"use_sim_time": True}]
            )
        ]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        #output="log",
        arguments=["-d", rviz_config_file],
        parameters=generate_parameters(),
        condition=IfCondition(start_rviz),
    )
    

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("ariac_moveit_config"),
                "/launch",
                "/ariac_robots_moveit.launch.py",
            ]
        )
    )

    nodes_to_start = [
        demo_cpp,
        delay_timer1,
        delay_timer2,
        rviz_node,
        moveit
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz", default_value="false", description="start rviz node?"
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )