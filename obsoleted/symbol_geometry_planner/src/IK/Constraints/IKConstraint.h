#ifndef IKCONSTRAINT_H
#define IKCONSTRAINT_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>

namespace IK{
  class IKConstraint
  {
  public:
    // QPで用いる
    // 必ずcalc_error -> calc_jacobian -> calc_jacobianineq -> calc_min/maxineqの順で呼ぶので，干渉計算などは最初に呼ばれる関数でまとめて行うこと
    // 等式制約のエラーを返す.
    virtual const Eigen::VectorXd& calc_error ();
    // 等式制約のヤコビアンを返す. bodyのroot6dof+全関節が変数
    virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calc_jacobian (const std::vector<cnoid::Body*>& bodies);
    // 不等式制約のmin値を返す
    virtual const Eigen::VectorXd& calc_minineq ();
    // 不等式制約のmax値を返す
    virtual const Eigen::VectorXd& calc_maxineq ();
    // 不等式制約のヤコビアンを返す
    virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calc_jacobianineq (const std::vector<cnoid::Body*>& bodies);

    // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

    // gradient(-ヤコビアン^T*エラー)を返す

    virtual std::vector<cnoid::SgNodePtr>& getDrawOnObjects();

    void set_debug_level(int _debuglevel){debuglevel = _debuglevel;}
  protected:
    // bodiesが同じかどうか
    bool is_bodies_same(const std::vector<cnoid::Body*>& bodies1,const std::vector<cnoid::Body*>& bodies2);

    int debuglevel;

    Eigen::VectorXd error;
    std::vector<cnoid::Body*> jacobian_bodies;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian;
    Eigen::VectorXd minineq;
    Eigen::VectorXd maxineq;
    std::vector<cnoid::Body*> jacobianineq_bodies;
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobianineq;
    std::vector<cnoid::SgNodePtr> drawOnObjects;
  };
}

#endif
