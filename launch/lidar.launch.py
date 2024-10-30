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

def set_sdf_pose(sdf_path, pose_dict):
    tree = xmlET.parse(sdf_path)
    root = tree.getroot()
    pose = root.find(".//link/pose")

    print(pose.text)
    new_pose = f"{pose_dict['x']} {pose_dict['y']} {pose_dict['z']} {pose_dict['roll']} {pose_dict['pitch']} {pose_dict['yaw']}"
    
    pose.text = new_pose
    
    print(pose.text)

    sdf_fname = os.path.basename(sdf_path)

    tmp_file_path = os.path.join("/tmp/",sdf_fname)
    
    tree.write(tmp_file_path)
    
    return tmp_file_path



def randomize_pose_for_class(inner_r, outer_r):
    dist = random.uniform(inner_r, outer_r)
    theta = random.uniform(0, np.pi*2)
    
    pose = {}
        
    pose['x'] = dist * np.cos(theta)
    pose['y'] = dist * np.sin(theta)
    pose['z'] = random.uniform(-0.1, 0.1)

    pose['yaw'] = random.uniform(0, np.pi*2)
    pose['roll'] = random.uniform(-0.01, 0.01)
    pose['pitch'] = random.uniform(-0.01, 0.01)

    return pose
    
def randomize_pose_for_lidar():
    pose = {}
    pose['x'] = random.uniform(-0.5, 0.5)
    pose['y'] = random.uniform(-0.5, 0.5)
    pose['z'] = random.uniform(0.5, 2.0)

    pose['yaw'] = random.uniform(-0.2,0.2)
    pose['roll'] = random.uniform(-0.2,0.2)
    pose['pitch'] = random.uniform(-0.2,0.2)
    
    return pose



def spawn_multiple(fname, base_name, n_units):
    tree = xmlET.parse(fname)
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

    lidar_model = "ouster_os1"
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
    return LaunchDescription([gz_sim, spawn_lidar, lidar_bridge, lidar_static_transform] + target_spawners)


if __name__ == "__main__":
    lidae_sim_dir = get_package_share_directory("lidar_sim")
    
    sdf_path = os.path.join(lidae_sim_dir,"models","fendt.sdf")

    spawn_multiple(sdf_path, "fendt", 3)
