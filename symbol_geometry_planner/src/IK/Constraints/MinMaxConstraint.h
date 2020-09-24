#ifndef MINMAXCONSTRAINT_H
#define MINMAXCONSTRAINT_H

#include <cnoid/BodyItem>

class MinMaxConstraint
{
public:
  virtual double min_angle ();
  virtual double max_angle ();
};

#endif
