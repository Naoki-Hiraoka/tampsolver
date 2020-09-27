#include "Util.h"

namespace IK{
  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(const cnoid::BodyItemPtr robot){
    std::vector<std::shared_ptr<IKConstraint> > constraints;
    for(size_t j=0;j<robot->body()->numJoints();j++){
      constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->body()->joint(j)));
    }
    return constraints;
  }
}
