#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <cnoid/BodyItem>
#include "JointLimitTable.h"

namespace RobotConfig {
  class RobotConfig {
  public:
    RobotConfig (const cnoid::BodyItemPtr robot, const std::string& url);
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> >& get_joint_mm_tables() {return joint_mm_tables;}
  private:
    const cnoid::BodyItemPtr robot;
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> > joint_mm_tables;
  };
};

#endif
