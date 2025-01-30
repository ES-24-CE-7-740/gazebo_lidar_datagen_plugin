
import os

from ament_index_python.packages import get_package_share_directory, re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.descriptions import executable
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

import xml.etree.ElementTree as xmlET

import random 
import numpy as np
import json

from base_objects import BaseSimAsset
from lidar_settings import BaseLidarSettings


#########################################################
#################### configuration ######################
#########################################################

# must match the sdf file names 

target_copies = 1

dataset_size = 100

# must have matching sdf file
lidar_type = "ouster_os0"

dataset_global_path = "/ws/dataset_output/"

ground_copies = 3
tractor_copies = 1
combine_harvester_copies = 1
trailer_copies = 1
other_target_copies = 1



ground_mesh_path = "/simulation_assets/ground_slices/"
tractor_mesh_path = "/simulation_assets/tractors/"
combine_mesh_path = "/simulation_assets/combine_harvesters/"
trailer_mesh_path = "/simulation_assets/trailers/"
other_target_mesh_path = "/simulation_assets/other_targets/"    

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
    
    

def generate_sdf_files(asset_dir, settings, name_to_label_dict):
    mesh_assets = os.listdir(asset_dir)
    sdf_paths = []
    for mesh_asset in mesh_assets:
        model_name = mesh_asset.split('.')[0]
        model_type = settings['type']
        base_sdf = settings['base_sdf']
        print("creating sim asset")
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
        model_label_found = False
        for key in name_to_label_dict.keys():
            print(f"testing if key {key} is in {model_name}  ")
            if key in model_name:
                print(f"key {key} is in {model_name}  ")
                sim_asset.set_label(name_to_label_dict[key])
                model_label_found = True 
            elif not model_label_found:
                print("no model name matched to name label key")
                sim_asset.set_label(settings['label'])
        
        scale = np.random.uniform(settings['low_scale'], settings['upper_scale'])
        settings['individual_scales']['model_name'] = scale
        
        sim_asset.set_scale(scale)

        sim_asset.save_to_dir(dataset_global_path)

        sdf_paths.append(sim_asset.get_save_location())

    return sdf_paths 

def generate_lidar_sdf_file(settings):
    sdf_paths = []
    model_name = settings['name']
    base_sdf = settings['base_sdf']
    print("creating sim asset")
    lidar_asset = BaseLidarSettings(base_sdf, model_name)

    range_settings = settings['range']
    lidar_asset._set_range(range_settings['min'], range_settings['max'])

    noise_settings = settings['noise']
    lidar_asset._set_noise(noise_settings['mean'], noise_settings['stddev'])

    scan_fov_settings = settings['scan_fov']
    lidar_asset._set_scan_fov(scan_fov_settings['horizontal_samples'],
                              scan_fov_settings['horizontal_min_angle'],
                              scan_fov_settings['horizontal_max_angle'],
                              scan_fov_settings['vertical_samples'],
                              scan_fov_settings['vertical_min_angle'],
                              scan_fov_settings['vertical_max_angle']
                              )

    move_settings = settings['movement_parameters']
    lidar_asset.set_movement_parameters(min_dist          = move_settings['min_dist'],
                                        max_dist          = move_settings['max_dist'],
                                        z_low             = move_settings['z_low'],
                                        z_high            = move_settings['z_high'],
                                        roll              = move_settings['roll'],
                                        pitch             = move_settings['pitch'],
                                        yaw               = move_settings['yaw'],
                                        appearance_p      = move_settings['appearance_p'],
                                        distance_to_other = move_settings['distance_to_other'])
    
    

    lidar_asset.save_to_dir("/tmp/")

    sdf_paths.append(lidar_asset.get_save_location())

    return sdf_paths

def generate_spawners(sdf_paths, coppies):
    spawners = []
    for sdf_path in sdf_paths:
        base_name = xmlET.parse(sdf_path).getroot().find(".//model").attrib["name"]
        spawners = spawners + spawn_multiple(sdf_path, base_name, coppies)

    return spawners

def generate_launch_description():
    lidar_sim_dir = get_package_share_directory("lidar_sim")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    data_recorder_dir = get_package_share_directory("data_recorder")
    try:
        os.makedirs(dataset_global_path + "/labels")
        os.makedirs(dataset_global_path + "/points")
    except Exception as e:
        print(f"got exception when trying to make folders for dataset {e}")
    
    with open(os.path.join(lidar_sim_dir, "configs", "model_label_config.json"), "r") as f:
        name_to_label_dict = json.load(f)["full_label_dict"]    


    with open(os.path.join(lidar_sim_dir, "configs", "ground_config.json"), "r") as f:
        ground_settings = json.load(f)    
    with open(os.path.join(lidar_sim_dir, "configs", "tractor_config.json"), "r") as f:
        tractor_settings = json.load(f)    
    with open(os.path.join(lidar_sim_dir, "configs", "combine_harvester_config.json"), "r") as f:
        combine_harvester_settings = json.load(f)    
    
    with open(os.path.join(lidar_sim_dir, "configs", "trailer_config.json"), "r") as f:
        trailer_settings = json.load(f)
    with open(os.path.join(lidar_sim_dir, "configs", "other_target_config.json"), "r") as f:
        other_target_setting = json.load(f)


    with open(os.path.join(lidar_sim_dir, "configs", "lidar_settings.json"), "r") as f:
        lidar_settings = json.load(f)

    

    ground_settings['base_sdf'] = os.path.join(lidar_sim_dir, "models", "ground_slice_base.sdf")
    tractor_settings['base_sdf'] = os.path.join(lidar_sim_dir, "models", "tractor_base.sdf")
    combine_harvester_settings['base_sdf'] = os.path.join(lidar_sim_dir, "models", "combine_base.sdf")
    trailer_settings["base_sdf"] = os.path.join(lidar_sim_dir, "models", "trailer_base.sdf")
    other_target_setting["base_sdf"] = os.path.join(lidar_sim_dir, "models", "other_target_base.sdf")
    lidar_settings['base_sdf'] = os.path.join(lidar_sim_dir, "models", "lidar_on_mount.sdf")

    all_settings = {"ground_settings":ground_settings,
                    "tractor_settings":tractor_settings,
                    "combine_harvester_settings":combine_harvester_settings,
                    "trailer_settings":trailer_settings,
                    "lidar_settings":lidar_settings,
                    "tractor_copies":tractor_copies,
                    "ground_copies":ground_copies,
                    "combine_harvester_copies":combine_harvester_copies,
                    "trailer_copies":trailer_copies,
                    "other_target_copies":other_target_copies}

    config_dict = {'settings':all_settings,
                   'target_copies':target_copies,
                   'dataset_size':dataset_size,
                   'lidar_type':lidar_type,
                   'name_label_dict':name_to_label_dict}

    with open(dataset_global_path+"/info.json", "w") as outfile:
        json.dump(config_dict, outfile)
        

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'gz_args': os.path.join(lidar_sim_dir, 'models', 'empty_world.sdf') + ' -v 3' ' --headless-rendering' + ' -r' + ' --render-engine ogre2'}.items(),)



    ### spawn lidar ###
    lidar_sdf_path = generate_lidar_sdf_file(lidar_settings)
    spawn_lidar = Node(package='ros_gz_sim',
                               executable='create',
                               name='spawn_lidar',
                               arguments=['-name', lidar_type, '-file', lidar_sdf_path],
                       output='screen')

    lidar_bridge = Node(package='ros_gz_bridge',
                        executable = 'parameter_bridge',
                        name='scan_bridge',
                        arguments=["/scan/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
                        output = 'screen')

    # lidar_bridge_vis = Node(package='ros_gz_bridge',
    #                     executable = 'parameter_bridge',
    #                     name='scan_bridge2',
    #                     arguments=["/scan_2/points@sensor_msgs/msg/PointCloud2@ignition.msgs.PointCloudPacked"],
    #                     output = 'screen')

    
    ### spawn targets ###
    spawners = []
    
    ground_sdf_paths = generate_sdf_files(ground_mesh_path, ground_settings, name_to_label_dict)
    print("ground sdfs generated")
    tractor_sdf_paths = generate_sdf_files(tractor_mesh_path, tractor_settings, name_to_label_dict)
    print("tractor sdfs generated")
    combine_harvester_sdf_paths = generate_sdf_files(combine_mesh_path, combine_harvester_settings, name_to_label_dict)
    print("combine sdfs generated")
    trailer_sdf_paths = generate_sdf_files(trailer_mesh_path, trailer_settings, name_to_label_dict)
    print("trailer sdfs generated")
    other_target_sdf_paths = generate_sdf_files(other_target_mesh_path, other_target_setting, name_to_label_dict)
    print("other targets sdfs generated")



    print("---------sdf files generated----------")

    spawners += generate_spawners(ground_sdf_paths, ground_copies)
    spawners += generate_spawners(combine_harvester_sdf_paths, combine_harvester_copies)
    spawners += generate_spawners(tractor_sdf_paths, tractor_copies)
    spawners += generate_spawners(trailer_sdf_paths, trailer_copies)
    spawners += generate_spawners(other_target_sdf_paths, other_target_copies)

    ### lidar visiulization stuff ###
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
                                  lidar_type+"/lidar_link/gpu_lidar" 
                                  ])

    # make rviz here!
    rviz =Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [os.path.join(lidar_sim_dir, 'rviz', 'default.rviz')]]
        )

    ### dataset recorder ### 
    dataset_recorder = Node(
        package='data_recorder',
        executable='data_recorder',
        name="lidar_data_recorder",
        parameters=[
            {'dataset_size': dataset_size},
            {'dataset_path': dataset_global_path},
            {'labels': name_to_label_dict},
        ],
        output="screen"
    )



    return LaunchDescription([gz_sim, spawn_lidar, lidar_bridge, lidar_static_transform, dataset_recorder, rviz] + spawners)

    # return LaunchDescription([gz_sim, spawn_lidar, lidar_bridge, lidar_static_transform, dataset_recorder, rviz])
