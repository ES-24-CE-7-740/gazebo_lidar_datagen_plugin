#include "lidar_sim/random_scaler.hh"
#include <algorithm>
#include <cmath>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Rand.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Types.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/NodeShared.hh>
#include <ignition/msgs/model.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/vector3d.pb.h>
#include <ignition/msgs/visual.pb.h>
#include <iterator>
#include <memory>
#include <ostream>
#include <sdf/Element.hh>
#include <sdf/InterfaceModel.hh>
#include <sdf/Joint.hh>
#include <string>
#include <ignition/common/Console.hh>
#include <ignition/plugin/Register.hh>

#include "lidar_sim/joint_random_mover.hh"

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/components/Name.hh> 
#include <ignition/gazebo/Joint.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/transport/Node.hh>
#include <ignition/plugin.hh>
#include <ignition/math.hh>
#include <vector>



namespace lidar_sim{

class RandomScalerPrivate{
public:
  std::vector<double> x_range;
  std::vector<double> y_range;
  std::vector<double> z_range;
  gz::sim::Model model;
  const std::shared_ptr<const sdf::Element> sdf;

};

RandomScaler::RandomScaler() : data_ptr(std::make_unique<RandomScalerPrivate>() ){

}

RandomScaler::~RandomScaler() {
}

void RandomScaler::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr){
  this->data_ptr->model = gz::sim::Model(_entity);

  
  double x_range_min = 1, x_range_max = 1; // Default values
  if (_sdf->HasElement("x_range"))
  {
    std::string x_range = _sdf->Get<std::string>("x_range");
    sscanf(x_range.c_str(), "%lf %lf", &x_range_min, &x_range_max); // Parse x_range
  }
  this->data_ptr->x_range = {x_range_min, x_range_max};

  double y_range_min = 1, y_range_max = 1; // Default values
  if (_sdf->HasElement("y_range"))
  {
    std::string y_range = _sdf->Get<std::string>("y_range");
    sscanf(y_range.c_str(), "%lf %lf", &y_range_min, &y_range_max); // Parse y_range
  }
  this->data_ptr->y_range = {y_range_min, y_range_max};
  
  double z_range_min = 1, z_range_max = 1; // Default values
  if (_sdf->HasElement("z_range"))
  {
    std::string z_range = _sdf->Get<std::string>("z_range");
    sscanf(z_range.c_str(), "%lf %lf", &z_range_min, &z_range_max); // Parse z_range
  }
  this->data_ptr->z_range = {z_range_min, z_range_max};



}


void RandomScaler::PreUpdate(const gz::sim::UpdateInfo &_info, 
                             gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused){
    return;
  } 
  
  double new_x_scale = gz::math::Rand::DblUniform(this->data_ptr->x_range[0], this->data_ptr->x_range[1]);
  double new_y_scale = gz::math::Rand::DblUniform(this->data_ptr->y_range[0], this->data_ptr->y_range[1]);
  double new_z_scale = gz::math::Rand::DblUniform(this->data_ptr->z_range[0], this->data_ptr->z_range[1]);

  auto model = this->data_ptr->model;

  auto& sdf = _ecm.SetComponentData

  
  ignition::msgs::Model model_msg; 

  ignition::msgs::Vector3d* scale = new ignition::msgs::Vector3d();

  scale->set_x(new_x_scale);
  scale->set_y(new_y_scale);
  scale->set_z(new_z_scale);

  gz::sim::components::Component
  _ecm.SetComponentData<gz::>(const Entity _entity, const typename ComponentTypeT::Type &_data)

  // if (sdf->HasElement("scale")){
  //   ignmsg<<"sdf scale found"<<std::endl;
  // }

  
}
}


IGNITION_ADD_PLUGIN(
  lidar_sim::RandomScaler,
  gz::sim::System,
  lidar_sim::RandomScaler::ISystemConfigure,
  lidar_sim::RandomScaler::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(lidar_sim::RandomScaler, "lidar_sim::random_scaler")

















