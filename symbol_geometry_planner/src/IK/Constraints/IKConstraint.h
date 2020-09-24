#ifndef IKCONSTRAINT_H
#define IKCONSTRAINT_H

#include <cnoid/BodyItem>
#include <Eigen/Sparse>

class IKConstraint
{
public:
  // エラーを返す. QPで用いる
  virtual cnoid::VectorXd calc_error ();

  // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

  // ヤコビアンを返す. bodyitemのroot6dof+全関節が変数
  virtual Eigen::SparseMatrix<double> calc_jacobian (const std::vector<cnoid::BodyItemPtr>& bodyitems);

  // gradient(-ヤコビアン^T*エラー)を返す
};

#endif
