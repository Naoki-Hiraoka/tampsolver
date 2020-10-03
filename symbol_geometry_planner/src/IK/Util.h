#ifndef IK_UTIL_H
#define IK_UTIL_H

#include "Constraints/MinMaxJointConstraint.h"
#include "../RobotConfig/RobotConfig.h"

namespace IK{
  //全jointの関節角度上下限を作る
  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(const cnoid::Body* robot, RobotConfig::RobotConfig& config);
}
#endif