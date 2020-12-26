#include "Util.h"
#include <cnoid/EigenUtil>

namespace Utility{
  void fixLegToCoords(std::shared_ptr<RobotConfig::RobotConfig> config, const cnoid::Position& coords, const std::vector<std::string>& legs){
    cnoid::Body* robot = config->get_robot();

    cnoid::Position footcoords;
    footcoords.translation() = cnoid::Vector3::Zero();
    footcoords.linear() = cnoid::Matrix3::Identity();
    for(size_t i=0;i<legs.size();i++){
      cnoid::Position legcoords = config->get_endeffectors()[legs[i]]->getlink()->T() * config->get_endeffectors()[legs[i]]->getlocalpos();
      footcoords.translation() = (footcoords.translation() * i + legcoords.translation()) / (i+1);
      cnoid::Vector3 omega = cnoid::omegaFromRot(footcoords.linear().transpose() * legcoords.linear());
      if(omega.norm()!=0){
        footcoords.linear() = footcoords.linear() * cnoid::Matrix3(cnoid::AngleAxis(omega.norm()/(i+1),omega.normalized()));
      }
    }

    // rootcoords * trans = footcoords
    // X * rootcoords * trans = coords
    //
    // X =  coords * footcoords^-1
    robot->rootLink()->T() = coords * footcoords.inverse() * robot->rootLink()->T();

  }
}
