#include "IKConstraint.h"

namespace IK{
  // QPで用いる
  // 等式制約のエラーを返す.
  const Eigen::VectorXd& IKConstraint::calc_error (){
    return this->error;
  }

  // 等式制約のヤコビアンを返す. bodyのroot6dof+全関節が変数
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& IKConstraint::calc_jacobian (const std::vector<cnoid::Body*>& bodies){
    if(!this->is_bodies_same(bodies,this->jacobian_bodies)){
      this->jacobian_bodies = bodies;

      int num_variables = 0;
      for(size_t i=0;i<this->jacobian_bodies.size();i++){
        num_variables += 6 + this->jacobian_bodies[i]->numJoints();
      }
      this->jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,num_variables);
    }
    return this->jacobian;
  }

  // 不等式制約のmin値を返す
  const Eigen::VectorXd& IKConstraint::calc_minineq (){
    return this->minineq;
  }

  // 不等式制約のmax値を返す
  const Eigen::VectorXd& IKConstraint::calc_maxineq (){
    return this->maxineq;
  }

  // 不等式制約のヤコビアンを返す
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& IKConstraint::calc_jacobianineq (const std::vector<cnoid::Body*>& bodies){
    if(!this->is_bodies_same(bodies,this->jacobianineq_bodies)){
      this->jacobianineq_bodies = bodies;

      int num_variables = 0;
      for(size_t i=0;i<this->jacobianineq_bodies.size();i++){
        num_variables += 6 + this->jacobianineq_bodies[i]->numJoints();
      }
      this->jacobianineq = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,num_variables);
    }
    return this->jacobianineq;
  }

  // コスト(エラーの二乗和)を返す. 非線形最適化で用いる

  // gradient(-ヤコビアン^T*エラー)を返す

  std::vector<cnoid::SgNodePtr>& IKConstraint::getDrawOnObjects(){
    return this->drawOnObjects;
  }

  bool IKConstraint::is_bodies_same(const std::vector<cnoid::Body*>& bodies1,const std::vector<cnoid::Body*>& bodies2){
    if (bodies1.size() != bodies2.size() ) return false;
    for(size_t i=0;i<bodies1.size();i++){
      if (bodies1[i] != bodies2[i] ) return false;
    }
    return true;
  }
}
