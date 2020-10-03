#include "RobotConfig.h"
#include <yaml-cpp/yaml.h>

namespace RobotConfig {
  RobotConfig::RobotConfig (const cnoid::Body* _robot, const std::string& url):
    robot(_robot)
  {
    YAML::Node config = YAML::LoadFile(url);

    if(config["end_effectors"].IsDefined()) {
      //TODO
      //std::cerr << config["end_effectors"].as<std::string>() << std::endl;
    }

    if(config["collision_pair"].IsDefined()) {
      //TODO
      //std::cerr << config["collision_pair"].as<std::string>() << std::endl;
    }

    if(config["joint_limit_table"].IsDefined()) {
      readJointLimitTableFromProperties(this->joint_mm_tables,
                                        this->robot,
                                        config["joint_limit_table"].as<std::string>());
    }
  }
};
