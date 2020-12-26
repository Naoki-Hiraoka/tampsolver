#ifndef UTILITY_UTIL_H
#define UTILITY_UTIL_H

#include "../RobotConfig/RobotConfig.h"

namespace Utility {
  void fixLegToCoords(std::shared_ptr<RobotConfig::RobotConfig> config, const cnoid::Position& coords, const std::vector<std::string>& legs=std::vector<std::string>{"rleg","lleg"});
}

#endif
