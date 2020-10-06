#include "MinMaxJointConstraint.h"

namespace IK{
  MinMaxJointConstraint::MinMaxJointConstraint(const cnoid::Link* _joint, std::shared_ptr<RobotConfig::JointLimitTable> _minmaxtable):
    joint(_joint),
    minmaxtable(_minmaxtable),
    maxvel(0.1),
    llimit(-1e30),
    ulimit(1e30)
  {
    return;
  }

  Eigen::VectorXd MinMaxJointConstraint::calc_minineq () {
    Eigen::VectorXd min(1);
    double limit = std::max(this->joint->q_lower(),this->llimit);
    if (this->minmaxtable) limit = std::max(limit,this->minmaxtable->getLlimit());
    min << std::max(limit - this->joint->q(), -maxvel);
    return min;
  }

  Eigen::VectorXd MinMaxJointConstraint::calc_maxineq () {
    Eigen::VectorXd max(1);
    double limit = std::min(this->joint->q_upper(),this->ulimit);
    if (this->minmaxtable) limit = std::min(ulimit,this->minmaxtable->getUlimit());
    max << std::min(limit - this->joint->q(), maxvel);
    return max;
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> MinMaxJointConstraint::calc_jacobianineq (const std::vector<cnoid::Body*>& bodies) {
    int dim = 0;
    for(size_t i=0; i < bodies.size(); i++){
      dim += 6 + bodies[i]->numJoints();
    }

    std::vector<Eigen::Triplet<double> > tripletList;
    int idx = 0;
    for(size_t b=0;b<bodies.size();b++){
      if(bodies[b] == this->joint->body()){
        tripletList.push_back(Eigen::Triplet<double>(0,idx+6+this->joint->jointId(),1));
        break;
      }
      idx += 6 + bodies[b]->numJoints();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian(1,dim);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

    return jacobian;
  }
}
