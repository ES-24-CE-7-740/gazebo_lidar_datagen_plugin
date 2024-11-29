import xml.etree.ElementTree as xml
import os 

class BaseLidarSettings:
    def __init__(self, base_lidar_sdf_file, name):
        self.tree = xml.parse(base_lidar_sdf_file)
        self.root = self.tree.getroot()
        self.name = name
        self.model = self.root.find(".//model")
        self._set_name()

    def _set_name(self):
        self.model.attrib["name"] = self.name
        print(f"set name to {self.model.attrib['name']}")

    def _set_range(self, min_range, max_range):
        range = self.root.find(".//range")
        range.find("min").text = f"{min_range}"
        range.find("max").text = f"{max_range}"
        print("Updated range parameters")
        

    def _set_noise(self, mean, std_dev):
        noise = self.root.find(".//noise")
        noise.find("mean").text = f"{mean}"
        noise.find("stddev").text = f"{std_dev}"
        print("Updated the noise parameters")

    def _set_scan_fov(self, hori_samples, hori_min_angle, hori_max_angle, vert_samples, vert_min_angle, vert_max_angle):
        horizontal = self.root.find(".//horizontal")
        vertical = self.root.find(".//vertical")
        horizontal.find("samples").text = f'{hori_samples}'
        horizontal.find("min_angle").text = f'{hori_min_angle}'
        horizontal.find("max_angle").text = f'{hori_max_angle}'
        vertical.find("samples").text = f'{vert_samples}'
        vertical.find("min_angle").text = f'{vert_min_angle}'
        vertical.find("max_angle").text = f'{vert_max_angle}'
        print("Updated the scan FOV parameters")

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

if __name__ == "__main__":
    lidar_settings = BaseLidarSettings("/home/a/personal/sem7/3d_gauss/gazebo_lidar_sim_ws/src/lidar_sim/models/tractor_base.sdf", "/home/a/personal/sem7/3d_gauss/datasets/meshes/simulation_assets/tractors/massey_simplified.dae", "tractor")
    lidar_settings.set_label(99)
    lidar_settings.set_movement_parameters(1, 10, 1, 1, 1, -0.1, 0.1, 5, 0.5)
    lidar_settings.save_to_dir("/tmp/")
    print(f"saved at {lidar_settings.get_save_location()}")
        

