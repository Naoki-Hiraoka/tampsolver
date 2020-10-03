#include "Util.h"

namespace IK{
  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(const cnoid::Body* robot, RobotConfig::RobotConfig& config){
    std::vector<std::shared_ptr<IKConstraint> > constraints;
    for(size_t j=0;j<robot->numJoints();j++){
      if(config.get_joint_mm_tables().find(robot->joint(j)) != config.get_joint_mm_tables().end())
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->joint(j), config.get_joint_mm_tables()[robot->joint(j)]));
      else
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->joint(j), nullptr));
    }
    return constraints;
  }
}
