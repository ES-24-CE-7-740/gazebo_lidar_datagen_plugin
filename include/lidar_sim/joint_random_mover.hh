
#ifndef LIDAR_SIM__JOINT_RANDOM_MOVER_HH_
#define LIDAR_SIM__JOINT_RANDOM_MOVER_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <algorithm>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <memory>
#include <vector>

namespace lidar_sim
{

class JointRandomMoverPrivate;


class JointRandomMover:
  // This class is a system.
  public gz::sim::System,
  // This class also implements the ISystemPreUpdate, ISystemUpdate,
  // and ISystemPostUpdate interfaces.
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: JointRandomMover();
  public: ~JointRandomMover() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override; 

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;
  private: std::unique_ptr<JointRandomMoverPrivate> data_ptr; 
};



}
#endif

