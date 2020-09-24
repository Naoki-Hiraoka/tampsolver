#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <cnoid/BodyItem>
#include "Constraints/IKConstraint.h"
#include "Constraints/MinMaxConstraint.h"

class IKsolver
{
public:
  IKsolver(std::vector<cnoid::BodyItemPtr>& variables,
           std::vector<std::vector<std::shared_ptr<MinMaxConstraint> > >& minmaxs,
           std::vector<std::shared_ptr<IKConstraint> >& tasks,
           std::vector<std::shared_ptr<IKConstraint> >& constraints);

  // コストが増加したり，constraintsのエラーが閾値以上だとfalse
  virtual bool solve_one_loop();
};

#endif
