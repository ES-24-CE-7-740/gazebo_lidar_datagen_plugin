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


def spawn_multiple(fname, base_name, n_units):
    tree= xmlET.parse(fname)
    root = tree.getroot()
    model = root.find(".//model")
    
    spawners = []

    for n in range(n_units):
        model.attrib['name'] = f"{base_name}_{n}"
        tree.write(f"/tmp/{base_name}_{n}n.sdf")

        spawner = Node(package='ros_gz_sim',
                       executable='create',
                       name=f"spawn_{base_name}_{n}",
                       arguments=['-name', f"{base_name}_{n}", '-file', f"/tmp/{base_name}_{n}n.sdf"])
        spawners.append(spawner)

    return spawners
    


    
def generate_launch_description():
    lidar_sim_dir = get_package_share_directory("lidar_sim")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'gz_args': os.path.join(lidar_sim_dir, 'models', 'empty_world.sdf') + ' -v 4' ' --headless-rendering' + ' -r'}.items(),)

    lidar_model = "lidar_on_mount"
    lidar_sdf_path = os.path.join(lidar_sim_dir,"models",f"{lidar_model}.sdf")
    

    spawn_lidar = Node(package='ros_gz_sim',
                               executable='create',
                               name='spawn_lidar',
                               arguments=['-name', lidar_model, '-file', lidar_sdf_path],
                       output='screen')

    fendt_sdf_path = os.path.join(lidar_sim_dir,"models","fendt.sdf")


    spawn_fendt = Node(package='ros_gz_sim',                       
                               executable='create',
                               name='spawn_fendt',
                               arguments=['-name', "fendt", '-file', fendt_sdf_path],
                               output='screen')


    target_spawners = []

    targets_sdf_paths = os.path.join(lidar_sim_dir, "models", "targets")

    for target_sdf_path in os.listdir(targets_sdf_paths):
        target_spawners = target_spawners + spawn_multiple(os.path.join(targets_sdf_paths,target_sdf_path), target_sdf_path.split(".")[0], 2)
    


    lidar_bridge = Node(package='ros_gz_bridge',
                        executable = 'parameter_bridge',
                        name='scan_bridge',
                        arguments=["/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
                        output = 'screen')




    lidar_static_transform = Node(package='tf2_ros',
                                  executable = 'static_transform_publisher',
                                  name='lidar_tf_pub',
                                  arguments= [
                                  '0',
                                  '0',
                                  '1',
                                  '0',
                                  '0',
                                  '0',                 
                                  "world",
                                  lidar_model+"/lidar_link/gpu_lidar" 
                                  ])
    rviz =Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(lidar_sim_dir, 'rviz', 'default.rviz')]]
        )

    return LaunchDescription([gz_sim, spawn_lidar, lidar_bridge, lidar_static_transform, rviz])


if __name__ == "__main__":
    lidae_sim_dir = get_package_share_directory("lidar_sim")
    
    sdf_path = os.path.join(lidae_sim_dir,"models","fendt.sdf")

    spawn_multiple(sdf_path, "fendt", 3)
