#include "MinMaxJointConstraint.h"

MinMaxJointConstraint::MinMaxJointConstraint(const cnoid::Link* _joint):
  joint(_joint) {
  return;
}

// TODO min-max table
double MinMaxJointConstraint::min_angle () {
  return this->joint->q_lower();
}


double MinMaxJointConstraint::max_angle () {
  return this->joint->q_upper();
}
