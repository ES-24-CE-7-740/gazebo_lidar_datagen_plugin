import xml.etree.ElementTree as xml
import os 

class BaseSimAsset:
    def __init__(self, base_sdf_file, mesh_file,name):
        self.tree = xml.parse(base_sdf_file)
        self.mesh_file = mesh_file
        self.root = self.tree.getroot()
        self.name = name
        self.model = self.root.find(".//model")

        self._set_mesh_location()
        self._set_name()


    def _set_mesh_location(self):
        print("updating mesh uri")
        uri = self.root.find(".//uri")
        uri.text = self.mesh_file

    def _set_name(self):
        self.model.attrib["name"] = self.name
        print(f"set name to {self.model.attrib['name']}")

    def set_label(self, label_value):
        visual = self.root.find(".//visual")
        visual.find("laser_retro").text = f"{label_value}"
        # print(f"set label to {self.model.attrib['laser_raster']}")

    def set_scale(self, scale):
        mesh = self.root.find(".//mesh")
        mesh.find("scale").text = f"{scale} {scale} {scale}"

    def set_movement_parameters(self, min_dist, max_dist, roll, pitch, yaw, z_low, z_high, distance_to_other ,appearance_p):
        plugin = self.root.find(".//plugin")
        print("updating movement parameters")
        plugin.find("range").text = f"{min_dist} {max_dist}"
        plugin.find("z_range").text = f"{z_low} {z_high}"
        plugin.find("roll_range").text= f"-{roll} {roll}"
        plugin.find("pitch_range").text = f"-{pitch} {pitch}"
        plugin.find("yaw_range").text = f"-{yaw} {yaw}"
        plugin.find("min_dist").text = f"{distance_to_other}"
        plugin.find("appearance_p").text = f"{appearance_p}" 
        print("updated movement parameters") 

    def save_to_dir(self, dir):
        self.save_location = os.path.join(dir, f"{self.name}.sdf")
        self.tree.write(self.save_location)
        print(f"saved to {self.save_location}")

    def get_save_location(self):
        return self.save_location 

    def get_movement_parameters(self):
        pass 
    
    def get_string(self):
        pass  

if __name__ == "__main__":
    base_sim_asset = BaseSimAsset("/home/a/personal/sem7/3d_gauss/gazebo_lidar_sim_ws/src/lidar_sim/models/tractor_base.sdf", "/home/a/personal/sem7/3d_gauss/datasets/meshes/simulation_assets/tractors/massey_simplified.dae", "tractor")
    base_sim_asset.set_label(99)
    base_sim_asset.set_movement_parameters(1, 10, 1, 1, 1, -0.1, 0.1, 5, 0.5)
    base_sim_asset.set_scale(12)
    base_sim_asset.save_to_dir("/tmp/")
    print(f"saved at {base_sim_asset.get_save_location()}")
        

