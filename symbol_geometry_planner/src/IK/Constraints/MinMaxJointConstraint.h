#ifndef MINMAXJOINTCONSTRAINT_H
#define MINMAXJOINTCONSTRAINT_H

#include "MinMaxConstraint.h"

class MinMaxJointConstraint : public MinMaxConstraint
{
public:
  MinMaxJointConstraint(const cnoid::Link* joint);

  double min_angle () override;
  double max_angle () override;

private:
  const cnoid::Link* joint;
};

#endif
