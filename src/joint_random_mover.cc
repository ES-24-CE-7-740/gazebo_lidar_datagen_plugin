#include "lidar_sim/joint_random_mover.hh"
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
#include <iterator>
#include <memory>
#include <ostream>
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

namespace lidar_sim {

class JointRandomMoverPrivate{
public: std::vector<gz::sim::Entity> joint_entities;
  public: std::vector<std::vector<std::string>> joint_names;
  public: std::vector<std::string> joint_move_groups;
  public: std::set<std::string> move_groups_set;
  public: gz::sim::Model model;

};

JointRandomMover::JointRandomMover() : data_ptr(std::make_unique<JointRandomMoverPrivate>()){
  

}

JointRandomMover::~JointRandomMover() {
}


void JointRandomMover::Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr){
  this->data_ptr->model = gz::sim::Model(_entity);
  

  if (!this->data_ptr->model.Valid(_ecm))
  {
    ignerr << "JointPositionController plugin should be attached to a model "
           << "entity. Failed to initialize." << std::endl;
    return;
  }
  
  std::vector<std::string> joint_name_vec_g;
  // Get params from SDF
  auto sdf_element_name = _sdf->FindElement("joint_name");
  auto sdf_element_move_group = _sdf->FindElement("joint_move_group");
  while (sdf_element_name)
  {
    if (!sdf_element_name->Get<std::string>().empty())
    {
      joint_name_vec_g.push_back(sdf_element_name->Get<std::string>());
    }
    else
    {
      ignerr << "<joint_name> provided but is empty." << std::endl;
    }
    sdf_element_name = sdf_element_name->GetNextElement("joint_name");
  }
  if (joint_name_vec_g.empty())
  {
    ignerr << "Failed to get any <joint_name>." << std::endl;
return;
  }

  this->data_ptr->joint_names.push_back(joint_name_vec_g);

  while (sdf_element_move_group)
  {
    if (!sdf_element_move_group->Get<std::string>().empty())
    {
      this->data_ptr->joint_move_groups.push_back(sdf_element_move_group->Get<std::string>());
      this->data_ptr->move_groups_set.insert(sdf_element_move_group->Get<std::string>());
    }
    else
    {
      ignerr << "<joint_move_group> provided but is empty." << std::endl;
    }
    sdf_element_move_group = sdf_element_move_group->GetNextElement("joint_move_group");
  }
  if (this->data_ptr->joint_move_groups.empty())
  {
    ignerr << "Failed to get any <joint_move_group>." << std::endl;
    return;
  }

  // ignition::gazebo::Entity test_joint = this->data_ptr->model.JointByName(_ecm, "right_wheel_joint");
  
  // test_joint.ResetPosiont(_ecm, 0.5);
  

}



void JointRandomMover::PreUpdate(const gz::sim::UpdateInfo &_info, 
                             gz::sim::EntityComponentManager &_ecm) {
  if (_info.paused){
    return;
  } 
  
  auto joint_names = this->data_ptr->joint_names;

  
  for (std::vector<std::string> joint_name_inner_vec : joint_names){
    // igndbg << joint_name << std::endl; 
    

    auto joint = ignition::gazebo::Joint();
    joint.ResetEntity(this->data_ptr->model.JointByName(_ecm, joint_name_inner_vec.back()));

    auto joint_axis_sdf = joint.Axis(_ecm)->back();

    double upper_joint_limit = joint_axis_sdf.Upper();
    double lower_joint_limit = joint_axis_sdf.Lower();



    std::vector<double> rand_pos;
    rand_pos.push_back(gz::math::Rand::DblUniform(lower_joint_limit, upper_joint_limit));
    
    for(std::string joint_name : joint_name_inner_vec){
      //set joint to random pose here 
      auto joint = ignition::gazebo::Joint();
      joint.ResetEntity(this->data_ptr->model.JointByName(_ecm, joint_name));
      joint.ResetPosition(_ecm, rand_pos);
    }


  }

}



}


IGNITION_ADD_PLUGIN(
  lidar_sim::JointRandomMover,
  gz::sim::System,
  lidar_sim::JointRandomMover::ISystemConfigure,
  lidar_sim::JointRandomMover::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(lidar_sim::JointRandomMover, "lidar_sim::joint_random_mover")

