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

class random_mover_private{
  public: std::set<std::string> target_move_group; 
  public: std::set<std::string> tracker_move_group; 
};





random_mover::random_mover() : data_ptr(std::make_unique<random_mover_private>()){
}
 
random_mover::~random_mover() {
}


gz::math::Pose3d random_mover::generate_random_tracker_pose(double min_z, double max_z, double max_roll, double max_pitch) {
  double x = 0;
  double y = 0;
  double z = gz::math::Rand::DblUniform(min_z, max_z);
  double roll = gz::math::Rand::DblUniform(-max_roll, max_roll); 
  double pitch = gz::math::Rand::DblUniform(-max_pitch, max_pitch);
  double yaw = gz::math::Rand::DblUniform(0, 2 *M_PI);

  return gz::math::Pose3d(x, y, z, roll, pitch, yaw);
} 


std::vector<gz::math::Pose3d> random_mover::generate_random_target_poses(double min_dist, double max_dist, int n_poses) {
  // Generate random position values within the specified range
  std::vector<gz::math::Pose3d> pose_vec;

  for (int i = 0; i < n_poses; i++){
    igndbg << i <<std::endl;
    double distance = gz::math::Rand::DblUniform(min_dist, max_dist);
    double angle = gz::math::Rand::DblUniform(0, 2*M_PI);


    double x = std::cos(angle)*distance;
    double y = std::sin(angle)*distance;
    double z = gz::math::Rand::DblUniform(0, 0.1);
    
    // Generate random orientation values (optional, or keep orientation constant)
    double roll = gz::math::Rand::DblUniform(0, 0.1 * M_PI);
    double pitch = gz::math::Rand::DblUniform(0, 0.1 * M_PI);
    double yaw = gz::math::Rand::DblUniform(0, 2 * M_PI);



    pose_vec.push_back(gz::math::Pose3d(x, y, z, roll, pitch, yaw));
  }

  
  return pose_vec;
}



void random_mover::Configure(const gz::sim::Entity &_entity,
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



void random_mover::PreUpdate(const gz::sim::UpdateInfo &_info, 
                             gz::sim::EntityComponentManager &_ecm) {
  // Implement pre-update logic if needed
}


void random_mover::Update(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm){

  if (_info.paused) {
    return;
  }

  auto target_group = this->data_ptr->target_move_group;
  auto tracker_group = this->data_ptr->tracker_move_group;
  
  auto target_poses_vec = generate_random_target_poses(5, 20, target_group.size());


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


void random_mover::PostUpdate(const gz::sim::UpdateInfo &_info,
                              const gz::sim::EntityComponentManager &_ecm) {
  // ignmsg << "SampleSystem::PostUpdate" << std::endl;
}



}// namespace lidar_sim


IGNITION_ADD_PLUGIN(
  lidar_sim::random_mover,
  gz::sim::System,
  lidar_sim::random_mover::ISystemConfigure,
  lidar_sim::random_mover::ISystemPreUpdate,
  lidar_sim::random_mover::ISystemUpdate,
  lidar_sim::random_mover::ISystemPostUpdate)

IGNITION_ADD_PLUGIN_ALIAS(lidar_sim::random_mover, "ignition::gazebo::systems::random_mover") // Fixed semicolon
