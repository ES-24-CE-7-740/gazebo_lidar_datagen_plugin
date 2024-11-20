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



#########################################################
#################### configuration ######################
#########################################################

# must match the sdf file names 
selected_targets = ["fendt",
                    "fendt_combine",
                    "valtra",
                    "valtra_2_simplified"]

target_copies = 3

dataset_size = 1000

# must have matching sdf file
lidar_type = "ouster_os1"

dataset_global_path = "/tmp/02"

# this might be usefull later
label_dict = {'0':'none',
              '1':'tractor',
              '2':'combine_harvester'}



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
    try:
        os.mkdir(dataset_global_path + "/labels")
        os.mkdir(dataset_global_path + "/points")
    except Exception as e:
        print(f"got exception when trying to make folders for dataset {e}")

    config_dict = {'selected_targets':selected_targets,
                   'target_copies':target_copies,
                   'dataset_size':dataset_size,
                   'lidar_type':lidar_type,
                   'label_dict':label_dict}
    with open(dataset_global_path+"/info.json", "w") as outfile:
        json.dump(config_dict, outfile)
        


    lidar_sim_dir = get_package_share_directory("lidar_sim")
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    data_recorder_dir = get_package_share_directory("data_recorder")
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ), 
        # set it to use the empty world specified in the pgk 
        launch_arguments={'gz_args': os.path.join(lidar_sim_dir, 'models', 'empty_world.sdf') + ' -v 1' ' --headless-rendering' + ' -r'}.items(),)



    ### spawn lidar ###
    lidar_sdf_path = os.path.join(lidar_sim_dir,"models",f"{lidar_type}.sdf")
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


    
    ### spawn targets ###
    target_spawners = []
    targets_sdf_path = os.path.join(lidar_sim_dir, "models", "targets")
    
    for target in selected_targets:
        target_spawners = target_spawners + spawn_multiple(os.path.join(targets_sdf_path,target+".sdf"), target, target_copies)
    


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

    ### dataset recorder ### 
    dataset_recorder = Node(
        package='data_recorder',
        executable='data_recorder',
        name="lidar_data_recorder",
        parameters=[
            {'dataset_size': dataset_size},
            {'dataset_path': dataset_global_path},
            {'labels': label_dict},
        ],
        output="screen"
    )



    return LaunchDescription([gz_sim, spawn_lidar, lidar_bridge, lidar_static_transform, dataset_recorder] + target_spawners)

