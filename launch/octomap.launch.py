import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    octomap_server=Node(
        package = 'octomap_server',
        name = 'octomap_server',
        executable = 'octomap_server_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"frame_id"               : "map"},
            {"resolution"             : 0.1},
            {"sensor_model.max_range" : 5.0},
            {"frequency"    : 15.0},
            {"octomap_path" : "/home/eric/ros_ws/src/my_robot/maps/fr_campus.bt"}
        ],    
        remappings = [
            ("cloud_in", "sensors/pointcloud"),
            ("octomap_full", "nav_lite/map")
        ]    
    )

    ld.add_action(octomap_server)

    return ld
