#ifndef LIDAR_SIM__RANDOM_MOVER_HH_
#define LIDAR_SIM__RANDOM_MOVER_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <algorithm>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <vector>

namespace lidar_sim
{
  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
// plugins that want to send commands.
class random_mover_private;

class random_mover:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: random_mover();
 
    public: ~random_mover() override;

    public: std::vector<gz::math::Pose3d> generate_random_target_poses(double min_dist, double max_dist, int n_poses); 
    public: gz::math::Pose3d generate_random_tracker_pose(double min_z, double max_z, double max_roll, double max_pitch);
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &_eventMgr) override; 

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void Update(const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;
 
    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                const gz::sim::EntityComponentManager &_ecm) override;
    
    private: std::unique_ptr<random_mover_private> data_ptr;
  };
}
#endif
