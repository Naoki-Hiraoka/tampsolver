#ifndef IKCONSTRAINT_H
#define IKCONSTRAINT_H

#include <cnoid/BodyItem>
#include <Eigen/Sparse>

namespace IK{
  class IKConstraint
  {
  public:
    // QPで用いる
    // 等式制約のエラーを返す.
    virtual Eigen::VectorXd calc_error ();
    // 等式制約のヤコビアンを返す. bodyitemのroot6dof+全関節が変数
    virtual Eigen::SparseMatrix<double,Eigen::RowMajor> calc_jacobian (const std::vector<cnoid::BodyItemPtr>& bodyitems);
    // 不等式制約のmin値を返す
    virtual Eigen::VectorXd calc_minineq ();
    // 不等式制約のmax値を返す
    virtual Eigen::VectorXd calc_maxineq ();
    // 不等式制約のヤコビアンを返す
    virtual Eigen::SparseMatrix<double,Eigen::RowMajor> calc_jacobianineq (const std::vector<cnoid::BodyItemPtr>& bodyitems);

    // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

    // gradient(-ヤコビアン^T*エラー)を返す

    void set_debug_level(int _debuglevel){debuglevel = _debuglevel;}
  protected:
    int debuglevel;
  };
}

#endif
