#include "livox_lidar_plugin/LivoxLidar.hh"
#include <ignition/plugin/Register.hh>

#define DEG2RAD (M_PI / 180.0)

using namespace livox_lidar_plugin;

std::vector<LaserDirection> LoadLaserDirections(const std::string& filename, char delimiter = ',') {
    std::vector<LaserDirection> directions;
    std::ifstream file(filename);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return directions;
    }

    // skip header
    std::getline(file, line);

    // load time, azimuth and zenith
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      uint32_t time_ns;
      double   azimuth_rad, zenith_rad;

      if (delimiter == ',') {
        std::string token;
        std::getline(ss, token, ','); time_ns     = (uint32_t)( std::stod(token) * 1e9     );
        std::getline(ss, token, ','); azimuth_rad = (double)(   std::stod(token) * DEG2RAD );
        std::getline(ss, token, ','); zenith_rad  = (double)(   std::stod(token) * DEG2RAD );
      } else {
        ss >> time_ns >> azimuth_rad >> zenith_rad;
        continue;
      }
      
      while( azimuth_rad < -M_PI )
      {
        azimuth_rad += 2*M_PI;
      }

      while( azimuth_rad > M_PI )
      {
        azimuth_rad -= 2*M_PI;
      }
      
      while( zenith_rad < -M_PI )
      {
        zenith_rad += 2*M_PI;
      }

      while( zenith_rad > M_PI )
      {
        zenith_rad -= 2*M_PI;
      }      
        
      directions.push_back(
        { 
          time_ns,
          azimuth_rad,
          zenith_rad,          
          0
        }
      );
    }

    return directions;
}


// constructor
LivoxLidar::LivoxLidar()
{
  std::cerr << "[LivoxLidar] Plugin constructed." << std::endl;
}


// configure file
void LivoxLidar::Configure(const ignition::gazebo::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           ignition::gazebo::EntityComponentManager &_ecm,
                           ignition::gazebo::EventManager &)
{


  // read parameters
  if (_sdf->HasElement("sensor_name"))
  {
    _sensor_name = _sdf->Get<std::string>("sensor_name");
  }
  
  if (_sdf->HasElement("topic"))
  {
    _topic = _sdf->Get<std::string>("topic");
  }

  if (_sdf->HasElement("frame_id"))
  {
    _frame_id = _sdf->Get<std::string>("frame_id");
  }

  if (_sdf->HasElement("model"))
  {
    _model = _sdf->Get<std::string>("model");
  }

  if (_sdf->HasElement("samples"))
  {
    _samples = _sdf->Get<unsigned int>("samples");
  }

  if (_sdf->HasElement("downsample"))
  {
    _downsample = _sdf->Get<unsigned int>("downsample");
  }

  if (_sdf->HasElement("sensor_topic"))
  {
    _sensor_topic = _sdf->Get<std::string>("sensor_topic");
  }
  
  if (_sdf->HasElement("h_samples"))
  {
    _h_samples = _sdf->Get<unsigned int>("h_samples");
  }
  
  if (_sdf->HasElement("v_samples"))
  {
    _v_samples = _sdf->Get<unsigned int>("v_samples");
  }

  std::cerr << "[LivoxLidar] sensor_model : " << _model        << std::endl;

  std::cerr << "[LivoxLidar] sensor_name  : " << _sensor_name  << std::endl;
  std::cerr << "[LivoxLidar] topic        : " << _topic        << std::endl;
  std::cerr << "[LivoxLidar] frame_id     : " << _frame_id     << std::endl;
  std::cerr << "[LivoxLidar] samples      : " << _samples      << std::endl;
  std::cerr << "[LivoxLidar] downsample   : " << _downsample   << std::endl;

  std::cerr << "[LivoxLidar] sensor topic : " << _sensor_topic << std::endl;  
  std::cerr << "[LivoxLidar] h_sample     : " << _h_samples    << std::endl;
  std::cerr << "[LivoxLidar] v_sample     : " << _v_samples    << std::endl;


  // calc projection table
  std::string _pkg_dir  = ament_index_cpp::get_package_share_directory("livox_lidar_plugin");  
  std::string _csv_path = _pkg_dir + "/scan_mode/" + _model + ".csv";  
  std::cerr << _csv_path << std::endl;
  
  _dir_table = LoadLaserDirections( _csv_path );
  
  for( uint32_t i = 0 ; i < _dir_table.size() ; i++ )
  {
    float _az = _dir_table[i].azimath_rad;
    float _ze = _dir_table[i].zenith_rad;
    int   h   = round((_az - h_min) * (_h_samples - 1) / (h_max - h_min));
    int   v   = round((_ze - v_min) * (_v_samples - 1) / (v_max - v_min));
    _dir_table[i].idx = ( v * _h_samples + h ) % (_h_samples*_v_samples);
  }

  // initialize subscriber (LaserScan)
  bool result = _node.Subscribe(_sensor_topic, &LivoxLidar::OnLaserScan, this);
  if (!result)
  {
    std::cerr << "[LivoxLidar] Failed to subscribe to topic: " << _topic << std::endl;
  }
  
  // initialize ign publisher (CustomMsg)
  _pub = _node.Advertise<LivoxCustomMsg::CustomMsg>(_topic);
  if (!_pub)
  {
    std::cerr << "[LivoxLidar] Failed to advatise topic: " << _topic << std::endl;  
  }
  
}


// update function
void LivoxLidar::Update(const ignition::gazebo::UpdateInfo &_info, ignition::gazebo::EntityComponentManager &_ecm)
{
  this->_latestSimTime = ignition::msgs::Convert(_info.simTime);
}


// laserscan callback
void LivoxLidar::OnLaserScan(const ignition::msgs::LaserScan &_msg)
{
  if (_msg.ranges_size() == 0)
  {
    std::cerr << "[LivoxLidar] Empty scan received." << std::endl;
    return;
  }
  
  if (_dir_table.empty()) {
    std::cerr << "[LivoxLidar] ERROR: _dir_table is empty!" << std::endl;
    return;
  }  

  LivoxCustomMsg::CustomMsg msg;
  
  msg.set_frame_id(_frame_id);
  msg.mutable_stamp()->CopyFrom(_latestSimTime);
  msg.set_time_base(0);
  msg.set_lidar_id(1);

  uint32_t val_pts = 0;
  uint32_t i       = 0;
  uint32_t line    = (uint32_t)( _start_index / _dir_table.size() );
  for (i = 0 ; i < _samples ; i+=_downsample)
  {
    if( _start_index + i >= _dir_table.size())
    {
      break;
    }
  
    uint32_t __i  = ( _start_index + i ) % _dir_table.size();
    uint32_t _idx = _dir_table[__i].idx;
    float    _az  = _dir_table[__i].azimath_rad;
    float    _ze  = _dir_table[__i].zenith_rad;    
    float    _d   = _msg.ranges(_idx);
    
    if (std::isinf(_d))
    {
      continue;
    }
    
    float _x = _d * cos(_ze) * cos(_az);
    float _y = _d * cos(_ze) * sin(_az);
    float _z = _d * sin(_ze);
  
    LivoxCustomMsg::CustomPoint *point = msg.add_points();
    point->set_x(_x);
    point->set_y(_y);
    point->set_z(_z);
    point->set_offset_time(_dir_table[__i].time_ns);
    point->set_reflectivity(255);
    point->set_tag(0);
    point->set_line(line);
    val_pts++;
  }
  _start_index = (_start_index + _samples) % _dir_table.size();
  
  msg.set_point_num(val_pts);
  _pub.Publish(msg);
}


IGNITION_ADD_PLUGIN(
  livox_lidar_plugin::LivoxLidar,
  ignition::gazebo::System,
  ignition::gazebo::ISystemConfigure,
  ignition::gazebo::ISystemUpdate
)

IGNITION_ADD_PLUGIN_ALIAS(
  livox_lidar_plugin::LivoxLidar, 
  "livox_lidar_plugin::LivoxLidar"
)

