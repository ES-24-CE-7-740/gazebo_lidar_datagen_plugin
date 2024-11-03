import os

from ament_index_python.packages import get_package_share_directory, re

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xml.etree.ElementTree as xmlET

import random 
import numpy as np



def generate_launch_description():
    lidar_sim_dir = get_package_share_directory("lidar_sim")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'gz_args': os.path.join(lidar_sim_dir, 'models', 'empty_world.sdf') + ' -v 4' ' --headless-rendering' + ' -r'}.items(),)

    
    sdf_path = os.path.join(lidar_sim_dir,"models","targets","fendt_joint_version.sdf")
    

    spawn_fendt = Node(package='ros_gz_sim',
                               executable='create',
                               name='spawn_lidar',
                               arguments=['-name', "fendt_vj", '-file', sdf_path],
                       output='screen')

    return LaunchDescription([gz_sim, spawn_fendt])


