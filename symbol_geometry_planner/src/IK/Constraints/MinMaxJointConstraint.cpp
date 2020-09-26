#include "MinMaxJointConstraint.h"

namespace IK{
  MinMaxJointConstraint::MinMaxJointConstraint(const cnoid::Link* _joint):
    joint(_joint) {
    return;
  }

  // TODO min-max table
  Eigen::VectorXd MinMaxJointConstraint::calc_minineq () {
    Eigen::VectorXd min(1);
    min << this->joint->q_lower();
    return min;
  }
  Eigen::VectorXd MinMaxJointConstraint::calc_maxineq () {
    Eigen::VectorXd max(1);
    max << this->joint->q_upper();
    return max;
  }
}
