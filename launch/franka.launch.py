import os
import yaml  
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_path
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (
    OpaqueFunction,
    ExecuteProcess,
    TimerAction
)


def load_yaml(file_path):
    with open(file_path, "r") as file:
        return yaml.safe_load(file)


moveit_config = MoveItConfigsBuilder("moveit_resources_panda").to_moveit_configs()
rviz_config_file = os.path.join(
    get_package_share_path("franka_follow"), "config", "franka.rviz"
)

servo_yaml_path = os.path.join(
    get_package_share_path("moveit_servo"), "config", "panda_simulated_config.yaml"
)

servo_params = load_yaml(servo_yaml_path)
servo_params['drift_dimensions'] = [False, False, False, False, False, True]

def launch_setup(context):
    actions = []

    actions += [
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_config_file],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="both",
            parameters=[moveit_config.robot_description, {"use_sim_time": True}],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="virtual_joint_broadcaster",
            arguments=["0", "0", "0", "0", "0", "0", "world", "panda_link0"],
            output="screen",
        ),
        Node(
            package="moveit_servo",
            executable="servo_node_main",
            name="servo_node",
            parameters=[
                servo_params,
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                {"use_sim_time": True},
            ],
            output="screen",
        ),
        Node(
            package="franka_follow",
            executable="cube_follower",
        ),
        # Moveit needs to be triggered 
        TimerAction(
        period=4.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2", "service", "call",
                    "/servo_node/start_servo",
                    "std_srvs/srv/Trigger",
                    "{}"
                ],
                output="screen"
            )
        ]
    )
    ]

    return actions


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
