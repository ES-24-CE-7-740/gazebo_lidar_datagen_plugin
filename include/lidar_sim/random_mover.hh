#ifndef LIDAR_SIM__RANDOM_MOVER_HH_
#define LIDAR_SIM__RANDOM_MOVER_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <algorithm>
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <string>
#include <unordered_set>
#include <vector>

namespace lidar_sim
{

class GroupedPoseVec{
public:
  std::string move_group;
  std::vector<gz::math::Pose3d> poses;
  // Define operator< for use in std::set
  bool operator == (const GroupedPoseVec &other) const {
    return move_group == other.move_group;
  }
};

// Custom hash function for GroupedPoseVec
struct GroupedPoseVecHash {
  std::size_t operator()(const GroupedPoseVec &gpv) const {
    return std::hash<std::string>()(gpv.move_group);
  }
};

class PoseStorage{
public:
  static PoseStorage& Instance() {
    static PoseStorage instance;
    return instance;
  }
  
  std::vector<GroupedPoseVec>& get_poses(){
    return poses;
  }
  std::vector<std::string>& get_move_group_set(){
    return move_group_set;
  }

  void clear_poses(){
    int n_pose_groups = poses.size();
    
    for (int i = 0; i < n_pose_groups; i++){
      poses[i].poses.clear();
    }
  }
private:
  PoseStorage() = default;
  ~PoseStorage() = default;

  PoseStorage(const PoseStorage&) = delete;
  PoseStorage& operator=(const PoseStorage&) = delete;

  std::vector<GroupedPoseVec> poses;
  std::vector<std::string> move_group_set;

};

  // This is the main plugin's class. It must inherit from System and at least
  // one other interface.
  // Here we use `ISystemPostUpdate`, which is used to get results after
  // physics runs. The opposite of that, `ISystemPreUpdate`, would be used by
// plugins that want to send commands.
class RandomMoverPrivate;

class RandomMover:
    // This class is a system.
    public gz::sim::System,
    // This class also implements the ISystemPreUpdate, ISystemUpdate,
    // and ISystemPostUpdate interfaces.
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: RandomMover();
 
    public: ~RandomMover() override;

public: std::vector<gz::math::Pose3d> generate_random_target_poses();
public: gz::math::Pose3d generate_random_pose();


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
    
    private: std::unique_ptr<RandomMoverPrivate> data_ptr;
  };
}
#endif
