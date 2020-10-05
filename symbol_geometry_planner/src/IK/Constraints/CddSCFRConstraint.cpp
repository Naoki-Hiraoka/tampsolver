#include "CddSCFRConstraint.h"

namespace IK{
  CddSCFRConstraint::CddSCFRConstraint(cnoid::Body* robot, const std::vector<std::shared_ptr<RobotConfig::EndEffector> >& endeffectors):
    SCFRConstraint(robot,endeffectors)
  {
  }

  void CddSCFRConstraint::calcSCFR(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d){

    this->SCFR_M = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,2);
    this->SCFR_u = Eigen::VectorXd(0);
    this->SCFR_l = Eigen::VectorXd(0);
    return;
  }

}
