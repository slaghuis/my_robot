import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()

    camera_node=Node(
        package = 'camera_lite',
        name = 'camera',
        executable = 'camera_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"frame_width"  : 1920},
            {"frame_height" : 1080},
            {"device_id"    : 0},
            {"frequency"    : 15.0},
            {"frame_id"     : "camera"},
            {"reliability"  : "reliable"},
            {"history"      : "keep_last"},
            {"depth"        : 5}
        ]
    )
    picture_node=Node(
        package = 'camera_lite',
        name = 'picture_node',
        executable = 'picture_node',
        output="screen",
        emulate_tty=True
    )
    
    camera_distance_node=Node(
        package = 'camera_lite',
        name = 'distance_node',
        executable = 'distance_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"image_resolution_x"  : 1920},
            {"image_resolution_y"  : 1080},
            {"angle_of_view"       : 1.08559479}
       ]     
    )
    ld.add_action(camera_node)
    ld.add_action(picture_node)
    ld.add_action(camera_distance_node)
    

    return ld
