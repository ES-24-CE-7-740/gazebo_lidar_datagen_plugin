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
#include <vector>

namespace lidar_sim
{

class RandomMoverPrivate{
  public: std::set<std::string> target_move_group; 
  public: std::set<std::string> tracker_move_group; 
};



RandomMover::RandomMover() : data_ptr(std::make_unique<RandomMoverPrivate>()){
}
 
RandomMover::~RandomMover() {
}


gz::math::Pose3d RandomMover::generate_random_tracker_pose(double min_z, double max_z, double max_roll, double max_pitch) {
  double x = 0;
  double y = 0;
  double z = gz::math::Rand::DblUniform(min_z, max_z);
  double roll = gz::math::Rand::DblUniform(-max_roll, max_roll); 
  double pitch = gz::math::Rand::DblUniform(-max_pitch, max_pitch);
  double yaw = gz::math::Rand::DblUniform(0, 2 *M_PI);

  return gz::math::Pose3d(x, y, z, roll, pitch, yaw);
} 


gz::math::Pose3d generate_random_pose(double min_dist, double max_dist){

  double distance = gz::math::Rand::DblUniform(min_dist, max_dist);
  double angle = gz::math::Rand::DblUniform(0, 2*M_PI);


  double x = std::cos(angle)*distance;
  double y = std::sin(angle)*distance;
  double z = gz::math::Rand::DblUniform(0, 0.1);
  
  // Generate random orientation values (optional, or keep orientation constant)
  double roll = gz::math::Rand::DblUniform(0, 0.1 * M_PI);
  double pitch = gz::math::Rand::DblUniform(0, 0.1 * M_PI);
  double yaw = gz::math::Rand::DblUniform(0, 2 * M_PI);
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


std::vector<gz::math::Pose3d> RandomMover::generate_random_target_poses(double min_dist, double max_dist, int n_poses, double visible_threshold) {
  // Generate random position values within the specified range
  auto& pose_vec = PoseStorage::Instance().get_poses();

  igndbg<<"generating poses for " << n_poses << " targets"<<std::endl;
  for (int i = 0; i < n_poses; i++){
    igndbg << i <<std::endl;
    gz::math::Pose3d random_pose = generate_random_pose(min_dist, max_dist);
    // radomize if it should be visible
    if (visible_threshold < gz::math::Rand::DblUniform(0,1)){
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
        if (closest_distance_to_other < 7){
          random_pose = generate_random_pose(min_dist, max_dist);
          igndbg<<"target not clear"<<std::endl;
          igndbg<<closest_distance_to_other<<std::endl;
        }
        else {
          not_clear_of_others = false;
          igndbg<< "target clear" <<std::endl;
        }
      }  
      pose_vec.push_back(random_pose);
    }
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

  std::string move_group; 

  if (_sdf->Get<std::string>("move_group", move_group, "target")){
    ignmsg << "setting " << model_name << " to move group " << move_group <<std::endl;
    if (move_group == "target"){
      this->data_ptr->target_move_group.insert(model_name);
    }
    else if (move_group == "tracker") {
      this->data_ptr->tracker_move_group.insert(model_name); 
    }

  }


  

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

  auto target_group = this->data_ptr->target_move_group;
  auto tracker_group = this->data_ptr->tracker_move_group;
  int n_targets = 0;

  _ecm.Each<gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
      // igndbg << gz::sim::Model(_entity).Name(_ecm) << std::endl;
      if (target_group.count(_name->Data()) > 0){
        n_targets++;
      }
    return true;
    }
  );


  auto target_poses_vec = generate_random_target_poses(7, 20, 1, 0.5);


  gz::sim::Entity model_entity = gz::sim::kNullEntity; 

  _ecm.Each<gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
      // igndbg << gz::sim::Model(_entity).Name(_ecm) << std::endl;
      if (tracker_group.count(_name->Data()) > 0){
        igndbg << _name->Data() << std::endl;
        model_entity = _entity;
        auto model = gz::sim::Model(model_entity);
        model.SetWorldPoseCmd(_ecm,generate_random_tracker_pose(0.7, 2, 0.1, 0.1));

      }
      else if (target_group.count(_name->Data()) > 0) {
        igndbg << _name->Data() << std::endl;
        model_entity = _entity;
        auto model = gz::sim::Model(model_entity);
        
        auto pose = target_poses_vec.back();
        target_poses_vec.pop_back();

        model.SetWorldPoseCmd(_ecm, pose);

      
      }
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
