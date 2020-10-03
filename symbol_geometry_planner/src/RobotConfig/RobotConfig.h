#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <cnoid/Body>
#include <memory>
#include "JointLimitTable.h"

namespace RobotConfig {
  class RobotConfig {
  public:
    RobotConfig (const cnoid::Body* robot, const std::string& url);
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> >& get_joint_mm_tables() {return joint_mm_tables;}
  private:
    const cnoid::Body* robot;
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> > joint_mm_tables;
  };
};

#endif
