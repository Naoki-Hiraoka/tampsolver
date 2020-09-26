#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <cnoid/BodyItem>
#include "Constraints/IKConstraint.h"
#include "Constraints/MinMaxConstraint.h"
#include "OsqpEigenSolver.h"

class IKsolver
{
public:
  IKsolver(const std::vector<cnoid::BodyItemPtr>& variables,
           const std::vector<std::vector<std::shared_ptr<MinMaxConstraint> > >& minmaxs,
           const std::vector<std::shared_ptr<IKConstraint> >& tasks,
           const std::vector<std::shared_ptr<IKConstraint> >& constraints);

  // コストが増加したり，constraintsのエラーが閾値以上だとfalse
  virtual bool solve_one_loop();

  virtual bool solve_optimization();

private:
  const std::vector<cnoid::BodyItemPtr> variables;
  const std::vector<std::vector<std::shared_ptr<MinMaxConstraint> > > minmaxs;
  const std::vector<std::shared_ptr<IKConstraint> > tasks;
  const std::vector<std::shared_ptr<IKConstraint> > constraints;

  bool qp_initialized;
  OsqpEigenSolver osqpeigensolver;
};

#endif
