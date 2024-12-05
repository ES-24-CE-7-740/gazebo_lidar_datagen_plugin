#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Name.hh>
#include <iterator>
#include <memory>
#include <ostream>
#include <sdf/InterfaceModel.hh>
#include <string>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include "lidar_sim/random_mover.hh"

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Name.hh> // Ensure this is the right component
#include <ignition/transport/Node.hh>
#include <ignition/plugin.hh>
#include <ignition/math.hh>
#include <unordered_set>
#include <vector>

namespace lidar_sim
{

class RandomMoverPrivate{
public: 
  // std::set<std::string> target_move_group; 
  // std::set<std::string> tracker_move_group;
  // std::set<std::string> ground_move_group;
  std::string model_name;
  std::string move_group;
  std::vector<double> range;
  std::vector<double> z_range;
  std::vector<double> roll_range;
  std::vector<double> pitch_range;
  std::vector<double> yaw_range;
  double appearance_p;
  double min_dist; //minimum distance to others 

};



RandomMover::RandomMover() : data_ptr(std::make_unique<RandomMoverPrivate>()){
}
 
RandomMover::~RandomMover() {
}


// gz::math::Pose3d RandomMover::generate_random_tracker_pose() {
//   double x = 0;
//   double y = 0;
//   double z = gz::math::Rand::DblUniform(min_z, max_z);
//   double roll = gz::math::Rand::DblUniform(-max_roll, max_roll); 
//   double pitch = gz::math::Rand::DblUniform(-max_pitch, max_pitch);
//   double yaw = gz::math::Rand::DblUniform(0, 2 *M_PI);
//
//   return gz::math::Pose3d(x, y, z, roll, pitch, yaw);
// } 


gz::math::Pose3d RandomMover::generate_random_pose(){

  double distance = gz::math::Rand::DblUniform(data_ptr->range[0], data_ptr->range[1]);
  double angle = gz::math::Rand::DblUniform(0, 2*M_PI);


  double x = std::cos(angle)*distance;
  double y = std::sin(angle)*distance;
  double z = gz::math::Rand::DblUniform(data_ptr->z_range[0], data_ptr->z_range[1]);
  
  // Generate random orientation values (optional, or keep orientation constant)
  double roll = gz::math::Rand::DblUniform(data_ptr->roll_range[0], data_ptr->roll_range[1]);
  double pitch = gz::math::Rand::DblUniform(data_ptr->pitch_range[0], data_ptr->pitch_range[1]);
  double yaw = gz::math::Rand::DblUniform(data_ptr->yaw_range[0], data_ptr->yaw_range[1]);
  return  gz::math::Pose3d(x, y, z, roll, pitch, yaw);
}

double distance_between_poses(gz::math::Pose3d pose_a, gz::math::Pose3d pose_b){
  double x_a = pose_a.X();
  double y_a = pose_a.Y();
  double z_a = pose_a.Z();
  double x_b = pose_b.X();
  double y_b = pose_b.Y();
  double z_b = pose_b.Z();

  double distance = std::sqrt(std::pow(x_a - x_b, 2) + std::pow(y_a - y_b, 2) + std::pow(z_a - z_b, 2));
  igndbg<<"distance from dst func " <<distance<<std::endl;
  return distance;

}


std::vector<gz::math::Pose3d> RandomMover::generate_random_target_poses() {
  // Generate random position values within the specified range
  auto& grouped_pose_vecs = PoseStorage::Instance().get_poses();

  std::vector<gz::math::Pose3d> pose_vec;
  for (GroupedPoseVec gpv : grouped_pose_vecs){
    if (gpv.move_group == this->data_ptr->move_group){
      pose_vec = gpv.poses;
      igndbg << "using pose vec for move group "<<gpv.move_group<<std::endl;
    } 
    else {
      igndbg << "no pose vec found for move group" <<gpv.move_group<<std::endl;
    }
  }
  igndbg<<"recived pose vec size "<<pose_vec.size()<<std::endl;

  
  gz::math::Pose3d random_pose = generate_random_pose();
  // radomize if it should be visible
  if (data_ptr->appearance_p < gz::math::Rand::DblUniform(0,1)){
    random_pose.SetZ(-50); 
    pose_vec.push_back(random_pose);
    igndbg << "target not visible" <<std::endl;
  }
  else{
    bool not_clear_of_others = true;
  
    if (pose_vec.empty()){
      igndbg << "pose_vec is empty" << std::endl;
      not_clear_of_others = false;
    }

    while (not_clear_of_others) {
    
      double closest_distance_to_other = 99;
      igndbg << "pose vec size "<< pose_vec.size() << std::endl;
      for(const gz::math::Pose3d& pose : pose_vec){
        double dist = distance_between_poses(pose, random_pose);
        if (dist < closest_distance_to_other){
          closest_distance_to_other = dist;
        }
      }
      igndbg<<"closest_distance_to_other "<<closest_distance_to_other<<std::endl;
      if (closest_distance_to_other < this->data_ptr->min_dist){
        random_pose = generate_random_pose();
        igndbg<<"target not clear"<<std::endl;
        igndbg<<closest_distance_to_other<<std::endl;
      }
      else {
        not_clear_of_others = false;
        igndbg<< "target clear" <<std::endl;
      }
    }
    
    int n_move_groups = grouped_pose_vecs.size();
    
    for (int i = 0;i < n_move_groups; i++){
      if (grouped_pose_vecs[i].move_group == this->data_ptr->move_group){

        grouped_pose_vecs[i].poses.push_back(random_pose);
        igndbg << "adding pose to move group "<<grouped_pose_vecs[i].move_group<<std::endl;
      }  
    }

    pose_vec.push_back(random_pose);
  }
  

  igndbg<<"final pose vec size "<<pose_vec.size()<<std::endl;

  return pose_vec;
}



void RandomMover::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr){
  auto model = gz::sim::Model(_entity);
  
  std::string model_name = model.Name(_ecm); 
  
  this->data_ptr->model_name = model_name;

  std::string move_group; 

  std::vector<GroupedPoseVec>& grouped_pose_vecs = PoseStorage::Instance().get_poses();
  std::vector<std::string>& move_group_vec = PoseStorage::Instance().get_move_group_set();
  std::set<std::string> move_group_set(move_group_vec.begin(), move_group_vec.end());

  if (_sdf->Get<std::string>("move_group", move_group, "target")){
    ignmsg << "setting " << model_name << " to move group " << move_group <<std::endl;
    this->data_ptr->move_group = move_group;
    GroupedPoseVec move_group_pose_vec;
    move_group_pose_vec.move_group = move_group;

    if (move_group_set.count(move_group) < 1){
      grouped_pose_vecs.push_back(move_group_pose_vec);
      move_group_vec.push_back(move_group);
      ignmsg<<"made a mode groupe with name: "<<move_group<<std::endl;
    }
    
  }
  ignmsg<<"made move groups: "<<std::endl;
  for (std::string mvg_name : move_group_vec){
    ignmsg<<mvg_name<<" ";
  }
  ignmsg<<std::endl;



  // Read numerical parameters like ranges and probabilities
  double range_min = 7.0, range_max = 20.0; // Default values
  if (_sdf->HasElement("range"))
  {
    std::string range = _sdf->Get<std::string>("range");
    sscanf(range.c_str(), "%lf %lf", &range_min, &range_max); // Parse range
    ignmsg<<"setting range from sdf params "<< range_min <<", "<<range_max<<std::endl;
  }
  this->data_ptr->range = {range_min, range_max};

  double z_range_min = -0.1, z_range_max = 0.1; // Default values
  if (_sdf->HasElement("z_range"))
  {
    std::string z_range = _sdf->Get<std::string>("z_range");
    sscanf(z_range.c_str(), "%lf %lf", &z_range_min, &z_range_max); // Parse z_range
  }
  this->data_ptr->z_range = {z_range_min, z_range_max};

  double roll_range_min = -0.01, roll_range_max = 0.01; // Default values
  if (_sdf->HasElement("roll_range"))
  {
    std::string roll_range = _sdf->Get<std::string>("roll_range");
    sscanf(roll_range.c_str(), "%lf %lf", &roll_range_min, &roll_range_max); // Parse roll_range
  }
  this->data_ptr->roll_range = {roll_range_min, roll_range_max};

  double pitch_range_min = -0.01, pitch_range_max = 0.01; // Default values
  if (_sdf->HasElement("pitch_range"))
  {
    std::string pitch_range = _sdf->Get<std::string>("pitch_range");
    sscanf(pitch_range.c_str(), "%lf %lf", &pitch_range_min, &pitch_range_max); // Parse pitch_range
  }
  this->data_ptr->pitch_range = {pitch_range_min, pitch_range_max}; 

  double yaw_range_min = 0.0, yaw_range_max = 6.28; // Default values
  if (_sdf->HasElement("yaw_range"))
  {
    std::string yaw_range = _sdf->Get<std::string>("yaw_range");
    sscanf(yaw_range.c_str(), "%lf %lf", &yaw_range_min, &yaw_range_max); // Parse yaw_range
  }
  this->data_ptr->yaw_range = {yaw_range_min, yaw_range_max};

  // Example of extracting additional parameters
  double appearance_p = 0.5; // Default value
  if (_sdf->HasElement("appearance_p"))
    appearance_p = _sdf->Get<double>("appearance_p");
  this->data_ptr->appearance_p = appearance_p;


  double min_dist = 5.0; // Default value
  if (_sdf->HasElement("min_dist"))
    min_dist = _sdf->Get<double>("min_dist");
    ignmsg << "min dist set to "<<min_dist<<std::endl;
  this->data_ptr->min_dist = min_dist;

  

}



void RandomMover::PreUpdate(const gz::sim::UpdateInfo &_info, 
                             gz::sim::EntityComponentManager &_ecm) {
  // Implement pre-update logic if needed
}


void RandomMover::Update(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm){

  if (_info.paused) {
    return;
  }

    std::string move_group = this->data_ptr->move_group;

    auto target_poses_vec = generate_random_target_poses();


    gz::sim::Entity model_entity = gz::sim::kNullEntity; 

    _ecm.Each<gz::sim::components::Name>(
      [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
        // igndbg << gz::sim::Model(_entity).Name(_ecm) << std::endl;
        if (this->data_ptr->model_name == _name->Data()){
          if (move_group == "tracker"){
            igndbg << _name->Data() << std::endl;
            model_entity = _entity;
            auto model = gz::sim::Model(model_entity);
            model.SetWorldPoseCmd(_ecm,generate_random_pose());

          }
          else if (move_group == "target") {
            igndbg << _name->Data() << std::endl;
            model_entity = _entity;
            auto model = gz::sim::Model(model_entity);
            
            if (!target_poses_vec.empty()) {
              auto pose = target_poses_vec.back();
              target_poses_vec.pop_back();
              model.SetWorldPoseCmd(_ecm, pose);
            } else {
              igndbg << "target_poses_vec is empty; cannot pop back!" << std::endl;
            }
          }
          else if (move_group == "ground") {
            igndbg << _name->Data() << std::endl;
            model_entity = _entity;
            auto model = gz::sim::Model(model_entity);
            
            if (!target_poses_vec.empty()) {
              auto pose = target_poses_vec.back();
              target_poses_vec.pop_back();
              model.SetWorldPoseCmd(_ecm, pose);
            } else {
              igndbg << "target_poses_vec is empty; cannot pop back!" << std::endl;
            }
          }
        }
        // else {
        //   igndbg << _name->Data() << std::endl;
        //   model_entity = _entity;
        //   auto model = gz::sim::Model(model_entity);
        //   
        //   if (!target_poses_vec.empty()) {
        //     auto pose = target_poses_vec.back();
        //     target_poses_vec.pop_back();
        //     model.SetWorldPoseCmd(_ecm, pose);
        //   } else {
        //     igndbg << "target_poses_vec is empty; cannot pop back!" << std::endl;
        //   }
        // }
      
      return true;
      }
  );



}


void RandomMover::PostUpdate(const gz::sim::UpdateInfo &_info,
                              const gz::sim::EntityComponentManager &_ecm) {
  // ignmsg << "SampleSystem::PostUpdate" << std::endl;
  PoseStorage::Instance().clear_poses();
}



}// namespace lidar_sim


IGNITION_ADD_PLUGIN(
  lidar_sim::RandomMover,
  gz::sim::System,
  lidar_sim::RandomMover::ISystemConfigure,
  lidar_sim::RandomMover::ISystemPreUpdate,
  lidar_sim::RandomMover::ISystemUpdate,
  lidar_sim::RandomMover::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(lidar_sim::RandomMover, "ignition::gazebo::systems::random_mover", "lidar_sim::random_mover" ) // Fixed semicolon
