import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

   sensor_pointcloud = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/sensor_pointcloud.launch.py'])
      )
      
   octomap = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/octomap.launch.py'])
      )
 
   navigation_lite = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/navigation_lite.launch.py'])
      )
      
   drone = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/drone_sim.launch.py'])
      )
   camera_lite = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/camera.launch.py'])
      )
      
   coverage_planner = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/coverage_planner.launch.py'])
      )
      
   lander = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/lander.launch.py'])
      )   
      
   flight_control = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('my_robot'), 'launch'),
         '/flight_control.launch.py'])
      )   
   
   return LaunchDescription([
      drone,
      sensor_pointcloud,
      octomap,
      navigation_lite,
      coverage_planner,
      flight_control
   ])


