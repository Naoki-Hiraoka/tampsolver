#include "IKConstraint.h"

namespace IK{
  // QPで用いる
  // 等式制約のエラーを返す.
  Eigen::VectorXd IKConstraint::calc_error (){
    return Eigen::VectorXd();
  }

  // 等式制約のヤコビアンを返す. bodyitemのroot6dof+全関節が変数
  Eigen::SparseMatrix<double,Eigen::RowMajor> IKConstraint::calc_jacobian (const std::vector<cnoid::BodyItemPtr>& bodyitems){
    int num_variables = 0;
    for(size_t i=0;i<bodyitems.size();i++){
      num_variables += 6 + bodyitems[i]->body()->numJoints();
    }
    return Eigen::SparseMatrix<double,Eigen::RowMajor>(0,num_variables);
  }

  // 不等式制約のmin値を返す
  Eigen::VectorXd IKConstraint::calc_minineq (){
    return Eigen::VectorXd();
  }

  // 不等式制約のmax値を返す
  Eigen::VectorXd IKConstraint::calc_maxineq (){
    return Eigen::VectorXd();
  }

  // 不等式制約のヤコビアンを返す
  Eigen::SparseMatrix<double,Eigen::RowMajor> IKConstraint::calc_jacobianineq (const std::vector<cnoid::BodyItemPtr>& bodyitems){
    int num_variables = 0;
    for(size_t i=0;i<bodyitems.size();i++){
      num_variables += 6 + bodyitems[i]->body()->numJoints();
    }
    return Eigen::SparseMatrix<double,Eigen::RowMajor>(0,num_variables);
  }

  // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

  // gradient(-ヤコビアン^T*エラー)を返す
}
