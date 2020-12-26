#ifndef POSITIONCONSTRAINT_H
#define POSITIONCONSTRAINT_H

#include "IKConstraint.h"
#include "../../RobotConfig/EndEffector.h"
#include <cnoid/EigenUtil>
#include <cnoid/JointPath>
#include <iostream>

namespace IK{
  class PositionConstraint : public IKConstraint
  {
  public:
    //リンクAの部位とリンクBの部位を一致させる.リンクがnullならworld固定を意味する
    PositionConstraint(cnoid::Link* A_link, const cnoid::Position& A_localpos,
                       cnoid::Link* B_link, const cnoid::Position& B_localpos);

    PositionConstraint(const cnoid::Position& A_pos,
                       const cnoid::Position& B_pos);
    PositionConstraint(const cnoid::Position& A_pos,
                       cnoid::Link* B_link, const cnoid::Position& B_localpos);
    PositionConstraint(cnoid::Link* A_link, const cnoid::Position& A_localpos,
                       const cnoid::Position& B_pos);

    PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                       cnoid::Link* B_link, const cnoid::Position& B_localpos);
    PositionConstraint(cnoid::Link* A_link, const cnoid::Position& A_localpos,
                       std::shared_ptr<RobotConfig::EndEffector> B);
    PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                       std::shared_ptr<RobotConfig::EndEffector> B);

    PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                       const cnoid::Position& B_pos);
    PositionConstraint(const cnoid::Position& A_pos,
                       std::shared_ptr<RobotConfig::EndEffector> B);


    // エラーを返す. A-B. world系. QPで用いる
    const Eigen::VectorXd& calc_error () override;

    // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

    // ヤコビアンを返す. bodyのroot6dof+全関節が変数
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calc_jacobian (const std::vector<cnoid::Body*>& bodies) override;

    // gradient(-ヤコビアン^T*エラー)を返す

    std::vector<cnoid::SgNodePtr>& getDrawOnObjects() override;

    void setmaxvel(double _maxvel) {maxvel=_maxvel;}

  private:
    cnoid::Link* A_link;
    const cnoid::Position A_localpos;
    cnoid::Link* B_link;
    const cnoid::Position B_localpos;
    double maxvel;

    cnoid::SgLineSetPtr lines;

    cnoid::JointPath path_A;
    cnoid::JointPath path_B;
    cnoid::JointPath path_BA;

  };
}

#endif
