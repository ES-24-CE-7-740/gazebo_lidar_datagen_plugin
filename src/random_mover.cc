#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Name.hh>
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
random_mover::random_mover() {
}
 
random_mover::~random_mover() {
}



gz::math::Pose3d random_mover::GenerateRandomPose(double xMin, double xMax, double yMin, double yMax, double zMin, double zMax) {
  // Generate random position values within the specified range
  double x = gz::math::Rand::DblUniform(xMin, xMax);
  double y = gz::math::Rand::DblUniform(yMin, yMax);
  double z = gz::math::Rand::DblUniform(zMin, zMax);
  
  // Generate random orientation values (optional, or keep orientation constant)
  double roll = gz::math::Rand::DblUniform(0, 2 * M_PI);
  double pitch = gz::math::Rand::DblUniform(0, 2 * M_PI);
  double yaw = gz::math::Rand::DblUniform(0, 2 * M_PI);
  
  return gz::math::Pose3d(x, y, z, 0, 0, yaw);
}



void random_mover::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr){
  auto model = gz::sim::Model(_entity);
  
  igndbg << model.Name(_ecm) << std::endl;

  if (!model.Valid(_ecm))
  {
    ignerr << "Random Mover plugin should be attached to a model entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  ignition::math::Pose3d newPose(7,7,0,0,0,0);
  model.SetWorldPoseCmd(_ecm, newPose);


  _ecm.Each<gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
      igndbg << gz::sim::Model(_entity).Name(_ecm) << std::endl;
      return true;
    }
  );

}



void random_mover::PreUpdate(const gz::sim::UpdateInfo &_info, 
                             gz::sim::EntityComponentManager &_ecm) {
  // Implement pre-update logic if needed
  if (_info.paused) {
    return;
  }

  std::string model_name = "fendt";

  gz::sim::Entity model_entity = gz::sim::kNullEntity; 
  

  
  _ecm.EachNoCache<gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
      // igndbg << _name->Data() << std::endl;
      if (_name->Data() == model_name){
        model_entity = _entity;
        return true;
      }
      else {
        return false;
      }
    }
  );

}


void random_mover::Update(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm){

  if (_info.paused) {
    return;
  }

  std::string model_name = "fendt";

  gz::sim::Entity model_entity = gz::sim::kNullEntity; 

  _ecm.Each<gz::sim::components::Name>(
    [&](const gz::sim::Entity &_entity, const gz::sim::components::Name *_name) -> bool {
      igndbg << gz::sim::Model(_entity).Name(_ecm) << std::endl;
      if (_name->Data() == model_name){
        model_entity = _entity;
      }
      return true;
    }
  );

  auto model = gz::sim::Model(model_entity);
  


  model.SetWorldPoseCmd(_ecm, GenerateRandomPose(10,-10,10,-10,0,0));

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
