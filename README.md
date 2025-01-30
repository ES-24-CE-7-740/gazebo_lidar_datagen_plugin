## general information 
Gazebo Fortress plugin to use gazebo as a lidar sematic segmentation dataset generator. Uses the laser_retro for each mesh to annotate the point clouds. The plugins primary function is to move meshes to random positions according to some rules each simulation tick.
The simulation assestes are in general split into two, the targets and the tracker (the LiDAR). The LiDAR can be configured in the lidar configuration json file. for the targets there is a configuration file for each target type where the movement parameters can be set. These configuration files are used to create the sdf files which are constructed in the launch file. In the launch file, the type of targets can be configured by appending them to the spawners list. To run the complete simulation envirionment, the simulation assets must be sorted into directories where each directory corresponds to a configuration file.

![](mat/gazebo_lidar.gif)


## use of plugin in sdf
The plugin can also be used in an sdf file directly by adding it the the model as in the example
```
<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="model_name" canonical_link="base">
    <static> true </static>
    <link name="base">
      <pose relative_to="__model__"> 0 0 0 0 0 0 </pose>
      <visual name="model_name_visual">
        <laser_retro>2</laser_retro>
        <geometry>
         <mesh>
            <uri>models://meshes/model_name.dae</uri>
          <scale>1.5 1.5 1.5</scale>
         </mesh>
       </geometry>
        <material>
          <ambient>0.1 0.1 0.1</ambient>
          <diffuse>0.1 0.1 0.1</diffuse>
          <specular>0.1 0.1 0.1</specular>
        </material>
      </visual>
    </link>
    <plugin
      filename="librandom_mover.so"
      name="lidar_sim::random_mover">
      <move_group>target</move_group>
      <range>7.0 31.0</range>
      <z_range>-0.1 0.1</z_range>
      <roll_range>-0.01 0.01</roll_range>
      <pitch_range>-0.01 0.01</pitch_range>
      <yaw_range>0 6.28</yaw_range>
      <appearance_p>1.0</appearance_p>
      <min_dist>5</min_dist>
    </plugin>
  </model>
</sdf>
```
the plugin parameters are used to configure the movement rules

```move_group``` is the name of the move groupe, which is used when when the assets in same move groupe arrent allown to collide. An example of this would be having grass sections as induvidual meshes that moves to random positions and vehicle meshes that moves to random positions. In this example it would make sense to have a move group for the vehicles and a move group for the grass sections as the x, y distance between these are irrelevant, whereas the vehicles should have some distance to each other. 

```range``` is the maximum and minimum range to the center of the simulation environment, this is used to configure a uniform sample used to generate the vehicle position each simulation tick. Note here the random position in generated from a rotation around the z-axis and a distance from the origin. 

```z_range``` is the maximum and minimum height of the model sampled uniformly each simulation tick

```roll_range``` the maximum and minimum roll of the model sampled uniformly each simulation tick 

```pitch_range``` same as the roll range

```yaw_range``` same as the roll range

```appearance_p``` the probabelity of the model being part of the scene for a given simulation tick where 1 means that the model will always appear in the scene and 0 means that the model will never appear

```min_dist``` the minimum distance to other models in the same move group, if the inital sampeld coordinate is within range of an alrealy established model of the same move group, the position is re-sampled
