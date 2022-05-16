import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    
    coverage_planner_node=Node(
        package = 'coverage_planner',
        name = 'coverage_planner_node',
        executable = 'coverage_planner_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"overlap"        :  0.1},
            {"minimum_height" :  5.0},
            {"maximum_height" : 30.0}
        ]
    )

    photogrammetry_node=Node(
        package = 'coverage_planner',
        name = 'photogrammetry_node',
        executable = 'photogrammetry_node',
        output="screen",
        emulate_tty=True,
        parameters = [
            {"images_folder" : "./"}
        ]
    )
    
    ld.add_action(coverage_planner_node)
    ld.add_action(photogrammetry_node)

    return ld
