
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

from base_objects import BaseSimAsset

import json

sim_assets = "/home/a/personal/sem7/3d_gauss/datasets/meshes/simulation_assets/ground_slices/"


def generate_sdf_files(asset_dir, settings):
    mesh_assets = os.listdir(asset_dir)
    sdf_paths = []
    for mesh_asset in mesh_assets:
        model_name = mesh_asset.split('.')[0]
        model_type = settings['type']
        base_sdf = settings['base_sdf']
        sim_asset = BaseSimAsset(base_sdf, os.path.join(asset_dir, mesh_asset), model_name)
        move_settings = settings['movement_parameters']
        sim_asset.set_movement_parameters(min_dist          = move_settings['min_dist'],
                                          max_dist          = move_settings['max_dist'],
                                          z_low             = move_settings['z_low'],
                                          z_high            = move_settings['z_high'],
                                          roll              = move_settings['roll'],
                                          pitch             = move_settings['pitch'],
                                          yaw               = move_settings['yaw'],
                                          appearance_p      = move_settings['appearance_p'],
                                          distance_to_other = move_settings['distance_to_other'])
        
        sim_asset.set_label(settings['label'])
        
        scale = np.random.uniform(settings['low_scale'], settings['upper_scale'])
        settings['individual_scales']['model_name'] = scale
        
        sim_asset.set_scale(scale)

        sim_asset.save_to_dir("/tmp/")

        sdf_paths.append(sim_asset.get_save_location())

    return sdf_paths 



         

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
    

def generate_spawners(sdf_paths, coppies):
    spawners = []
    for sdf_path in sdf_paths:
        base_name = xmlET.parse(sdf_path).getroot().find(".//model").attrib["name"]
        spawners = spawners + spawn_multiple(sdf_path, base_name, coppies)

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
    

    with open(os.path.join(lidar_sim_dir, "configs", "ground_config.json"), "r") as f:
        ground_asset_settings = json.load(f)    
    
    ground_asset_settings['base_sdf'] = os.path.join(lidar_sim_dir, "models", "ground_slice_base.sdf")
    
    sdf_paths = generate_sdf_files(sim_assets, ground_asset_settings)
    
    print(f"generated sdf paths:\n---------------\n{sdf_paths}\n---------------")
    
    spawners = generate_spawners(sdf_paths,2)

    return LaunchDescription([gz_sim] + spawners)


if __name__ == "__main__":
    lidae_sim_dir = get_package_share_directory("lidar_sim")
    
    sdf_path = os.path.join(lidae_sim_dir,"models","fendt.sdf")

    spawn_multiple(sdf_path, "fendt", 3)
