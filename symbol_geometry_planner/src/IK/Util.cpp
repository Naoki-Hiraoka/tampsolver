#include "Util.h"

namespace IK{
  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(const cnoid::BodyItemPtr robot, RobotConfig::RobotConfig& config){
    std::vector<std::shared_ptr<IKConstraint> > constraints;
    for(size_t j=0;j<robot->body()->numJoints();j++){
      if(config.get_joint_mm_tables().find(robot->body()->joint(j)) != config.get_joint_mm_tables().end())
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->body()->joint(j), config.get_joint_mm_tables()[robot->body()->joint(j)]));
      else
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->body()->joint(j), nullptr));
    }
    return constraints;
  }
}
