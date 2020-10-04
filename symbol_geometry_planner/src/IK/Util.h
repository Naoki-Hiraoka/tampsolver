#ifndef IK_UTIL_H
#define IK_UTIL_H

#include "Constraints/MinMaxJointConstraint.h"
#include "Constraints/CollisionConstraint.h"
#include "../RobotConfig/RobotConfig.h"

namespace IK{
  //全linkpairの干渉回避制約を作る
  std::vector<std::shared_ptr<IKConstraint> > generateCollisionConstraints(cnoid::Body* robot, std::shared_ptr<RobotConfig::RobotConfig> config, const std::string& mode="VClip");

  //全jointの関節角度上下限を作る
  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(cnoid::Body* robot, std::shared_ptr<RobotConfig::RobotConfig> config);
}
#endif
