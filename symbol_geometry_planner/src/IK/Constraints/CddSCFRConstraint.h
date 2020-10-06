#ifndef CDDSCFRCONSTRAINT_H
#define CDDSCFRCONSTRAINT_H

#include "SCFRConstraint.h"

namespace IK{
  class CddSCFRConstraint : public SCFRConstraint
  {
  public:
    CddSCFRConstraint(cnoid::Body* robot, const std::vector<std::shared_ptr<RobotConfig::EndEffector> >& endeffectors);
  protected:
    //SCFR_M, SCFR_u, SCFR_lをセットする
    void calcProjection(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d) override;
    void updateSCFRlines() override;

    //SCFRのV-表現
    Eigen::MatrixXd V2;
    Eigen::MatrixXd R_nonneg2;
    Eigen::MatrixXd R_free2;
  };
}

#endif