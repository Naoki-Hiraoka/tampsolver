#include "MinMaxJointConstraint.h"

namespace IK{
  MinMaxJointConstraint::MinMaxJointConstraint(const cnoid::Link* _joint):
    joint(_joint) {
    return;
  }

  // TODO min-max table
  Eigen::VectorXd MinMaxJointConstraint::calc_minineq () {
    Eigen::VectorXd min(1);
    min << this->joint->q_lower() - this->joint->q();
    return min;
  }

  Eigen::VectorXd MinMaxJointConstraint::calc_maxineq () {
    Eigen::VectorXd max(1);
    max << this->joint->q_upper() - this->joint->q();
    return max;
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> MinMaxJointConstraint::calc_jacobianineq (const std::vector<cnoid::BodyItemPtr>& bodyitems) {
    int dim = 0;
    for(size_t i=0; i < bodyitems.size(); i++){
      dim += 6 + bodyitems[i]->body()->numJoints();
    }

    std::vector<Eigen::Triplet<double> > tripletList;
    int idx = 0;
    for(size_t b=0;b<bodyitems.size();b++){
      if(bodyitems[b]->body() == this->joint->body()){
        tripletList.push_back(Eigen::Triplet<double>(0,idx+6+this->joint->jointId(),1));
        break;
      }
      idx += 6 + bodyitems[b]->body()->numJoints();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian(1,dim);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

    return jacobian;
  }
}
