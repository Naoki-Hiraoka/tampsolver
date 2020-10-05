#ifndef SCFRCONSTRAINT_H
#define SCFRCONSTRAINT_H

#include "IKConstraint.h"
#include "../../RobotConfig/EndEffector.h"
#include <cnoid/EigenUtil>
#include <iostream>

namespace IK{
  class SCFRConstraint : public IKConstraint
  {
  public:
    //robotの重心をendeffectorによって定義されるSCFR内に位置させる.SCFRの形状を能動的に変えることはしない．初回にSCFRを一回計算したら後は使い回す．
    SCFRConstraint(cnoid::Body* robot, const std::vector<std::shared_ptr<RobotConfig::EndEffector> >& endeffectors);

    Eigen::VectorXd calc_minineq () override;
    Eigen::VectorXd calc_maxineq () override;
    Eigen::SparseMatrix<double,Eigen::RowMajor> calc_jacobianineq (const std::vector<cnoid::Body*>& bodies) override;
    std::vector<cnoid::SgNodePtr> getDrawOnObjects() override;

    void setmaxvel(double _maxvel) {maxvel=_maxvel;}

  protected:
    //SCFR_M, SCFR_u, SCFR_lをセットする
    virtual void calcSCFR(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d)=0;

    cnoid::Body* robot;
    std::vector<std::shared_ptr<RobotConfig::EndEffector> > endeffectors;

    double g;
    double maxvel;

    bool initial_p;
    Eigen::SparseMatrix<double,Eigen::RowMajor> SCFR_M;
    Eigen::VectorXd SCFR_u;
    Eigen::VectorXd SCFR_l;

    cnoid::SgLineSetPtr SCFRlines;
  };
}

#endif
