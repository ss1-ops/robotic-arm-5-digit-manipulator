from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def load_file(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    path = os.path.join(pkg, file_path)
    with open(path, "r") as f:
        return f.read()

def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    path = os.path.join(pkg, file_path)
    with open(path, "r") as f:
        return yaml.safe_load(f)

def generate_launch_description():
    robot_description = {"robot_description": load_file("moveo_description", "urdf/moveo_clean.urdf")}
    robot_description_semantic = {"robot_description_semantic": load_file("moveo_moveit_config", "config/moveo.srdf")}
    kinematics = {"robot_description_kinematics": load_yaml("moveo_moveit_config", "config/kinematics.yaml")}
    joint_limits = {"robot_description_planning": load_yaml("moveo_moveit_config", "config/joint_limits.yaml")}
    moveit_controllers = load_yaml("moveo_moveit_config", "config/moveit_controllers.yaml")

    # MoveIt 2 Jazzy uses planning_pipelines (list) + per-pipeline namespace
    planning_pipelines = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugins": ["ompl_interface/OMPLPlanner"],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
            "arm": {
                "default_planner_config": "RRTConnect",
            },
        },
    }

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics,
            joint_limits,
            moveit_controllers,
            planning_pipelines,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"monitor_dynamics": False},
        ],
    )

    return LaunchDescription([move_group])
