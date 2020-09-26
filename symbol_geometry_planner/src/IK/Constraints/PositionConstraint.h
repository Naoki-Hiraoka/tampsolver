#ifndef POSITIONCONSTRAINT_H
#define POSITIONCONSTRAINT_H

#include "IKConstraint.h"
#include <cnoid/EigenUtil>
#include <iostream>

namespace IK{
  class PositionConstraint : public IKConstraint
  {
  public:
    //リンクAの部位とリンクBの部位を一致させる.リンクがnullならworld固定を意味する
    PositionConstraint(const cnoid::Link* A_link, const cnoid::Position& A_localpos,
                       const cnoid::Link* B_link, const cnoid::Position& B_localpos);

    // エラーを返す. A-B. world系. QPで用いる
    Eigen::VectorXd calc_error () override;

    // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

    // ヤコビアンを返す. bodyitemのroot6dof+全関節が変数
    Eigen::SparseMatrix<double,Eigen::RowMajor> calc_jacobian (const std::vector<cnoid::BodyItemPtr>& bodyitems) override;

    // gradient(-ヤコビアン^T*エラー)を返す

  private:
    const cnoid::Link* A_link;
    const cnoid::Position A_localpos;
    const cnoid::Link* B_link;
    const cnoid::Position B_localpos;
  };
}

#endif
