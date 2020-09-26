#include "IKsolver.h"

IKsolver::IKsolver(const std::vector<cnoid::BodyItemPtr>& _variables,
                   const std::vector<std::vector<std::shared_ptr<MinMaxConstraint> > >& _minmaxs,
                   const std::vector<std::shared_ptr<IKConstraint> >& _tasks,
                   const std::vector<std::shared_ptr<IKConstraint> >& _constraints):
  variables(_variables),
  minmaxs(_minmaxs),
  tasks(_tasks),
  constraints(_constraints),
  qp_initialized(false)
{

}

// コストが増加したり，constraintsのエラーが閾値以上だとfalse
bool IKsolver::solve_one_loop(){
  if(!qp_initialized){
    
    qp_initialized = true;
  }


  return true;
}

bool IKsolver::solve_optimization() {
  //TODO
  qp_initialized = false;
  return true;
}
