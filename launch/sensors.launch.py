import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()
         
    sonar_sensor=Node(
        package = 'sonar',
        name = 'sonar_node',
        executable = 'sonar_node',
        output="screen",
        emulate_tty=True
    )
    
    lidar_sensor=Node(
        package = 'garmin_lidar',
        name = 'lidar_node',
        executable = 'lidar_node',
        output="screen",
        emulate_tty=True
    )
     
    altitude_sensor=Node(
        package = 'vl53l1x',
        name = 'vl53l1x_node',
        executable = 'vl53l1x_node',
        output="screen",
        emulate_tty=True,
        parameters=[
            {"timeout": 500},
            {"timing_budget": 50000},
            {"frequency": 25.0}
        ]
    ) 
    

    ld.add_action(sonar_sensor)
    ld.add_action(altitude_sensor)
    ld.add_action(lidar_sensor)

    return ld
