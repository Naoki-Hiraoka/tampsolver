#ifndef MINMAXJOINTCONSTRAINT_H
#define MINMAXJOINTCONSTRAINT_H

#include "IKConstraint.h"

namespace IK{
  class MinMaxJointConstraint : public IKConstraint
  {
  public:
    MinMaxJointConstraint(const cnoid::Link* joint);

    Eigen::VectorXd calc_minineq () override;
    Eigen::VectorXd calc_maxineq () override;

  private:
    const cnoid::Link* joint;
  };
}

#endif
