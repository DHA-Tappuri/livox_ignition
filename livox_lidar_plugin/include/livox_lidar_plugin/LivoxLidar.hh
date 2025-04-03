#ifndef LIVOX_LIDAR_HH_
#define LIVOX_LIDAR_HH_

#include <memory>
#include <string>

#include <ignition/gazebo/System.hh>
#include <ignition/transport/Node.hh>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ignition/msgs/laserscan.pb.h>
#include "CustomMsg.pb.h"

#include <ignition/sensors/Manager.hh>
#include <ignition/sensors/GpuLidarSensor.hh>

namespace livox_lidar_plugin
{

  struct LaserDirection {
    uint32_t time_ns;
    float    azimath_rad;
    float    zenith_rad;
    uint32_t idx;
  };


  class LivoxLidar : public ignition::gazebo::System,
                     public ignition::gazebo::ISystemConfigure,
                     public ignition::gazebo::ISystemUpdate
  {
  public:
    LivoxLidar();
    ~LivoxLidar() override = default;

    void Configure(const ignition::gazebo::Entity &_entity,
                   const std::shared_ptr<const sdf::Element> &_sdf,
                   ignition::gazebo::EntityComponentManager &_ecm,
                   ignition::gazebo::EventManager &_eventMgr) override;

    void Update(const ignition::gazebo::UpdateInfo &_info,
                ignition::gazebo::EntityComponentManager &_ecm) override;

  private:
    // parameters
    void OnLaserScan(const ignition::msgs::LaserScan &_msg);

    ignition::transport::Node            _node;
    ignition::transport::Node::Publisher _pub;
    ignition::msgs::Time                 _latestSimTime;
    
    std::vector<LaserDirection>          _dir_table;
    
    std::string  _topic        = "/livox_points";
    std::string  _frame_id     = "livox_frame";
    std::string  _sensor_name  = "livox_lidar";
    std::string  _model        = "mid360";
    unsigned int _samples      = 24000;
    unsigned int _downsample   = 1;

    std::string  _sensor_topic = "/livox_lidar_sensor/scan";    
    unsigned int _h_samples    = 1440;
    unsigned int _v_samples    = 1440;
    
    float    h_min      = -M_PI;
    float    h_max      =  M_PI;
    float    v_min      = -M_PI;
    float    v_max      =  M_PI;
    
    unsigned int _start_index = 0;
  };
}

#endif

