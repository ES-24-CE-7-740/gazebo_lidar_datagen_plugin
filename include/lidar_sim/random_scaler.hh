#ifndef LIDAR_SIM__RANDOM_SCALER_HH_
#define LIDAR_SIM__RANDOM_SCALER_HH_

// The only required include in the header is this one.
// All others will depend on what your plugin does.
#include <gz/math/Pose3.hh>
#include <gz/sim/System.hh>
#include <memory>


#include <gz/math/Vector3.hh>

namespace custom_components
{
    struct ScaleTag {}; // Unique identifier for the Scale component
    using Scale = gz::sim::components::Component<gz::math::Vector3d, ScaleTag>;
}


namespace lidar_sim
{

class RandomScalerPrivate;


class RandomScaler:
  // This class is a system.
  public gz::sim::System,
  // This class also implements the ISystemPreUpdate, ISystemUpdate,
  // and ISystemPostUpdate interfaces.
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
  public: RandomScaler();
  public: ~RandomScaler() override;

  public: void Configure(const gz::sim::Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &_ecm,
                         gz::sim::EventManager &_eventMgr) override; 

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &_ecm) override;
  private: std::unique_ptr<RandomScalerPrivate> data_ptr; 
};



}
#endif

