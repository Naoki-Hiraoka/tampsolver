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
    Eigen::SparseMatrix<double,Eigen::RowMajor> calc_jacobianineq (const std::vector<cnoid::BodyItemPtr>& bodyitems) override;

  private:
    const cnoid::Link* joint;
  };
}

#endif
