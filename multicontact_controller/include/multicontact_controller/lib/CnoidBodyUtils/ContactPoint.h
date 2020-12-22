#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_CONTACT_POINT_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_CONTACT_POINT_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <Eigen/Sparse>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    class ContactPoint {
    public:
      std::string name() const { return name_; }
      std::string& name() { return name_; }

      cnoid::Link* const parent() const { return parent_; }
      cnoid::Link*& parent() { return parent_; }

      cnoid::Position T_local() const { return T_local_; }
      cnoid::Position& T_local() {return T_local_; }

      // local系. ContactPointまわり
      cnoid::Vector6 F() const { return F_; }
      cnoid::Vector6& F() {return F_; }

      // これらの関数はクラスから分離してもよいが、配列などをキャッシュしたいので
      virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian() {
        return calcJacobian(parent_, T_local_);
      }
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian(cnoid::Link* parent, cnoid::Position& T_local);//world系,contactpoint周り
      virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv() {
        return calcRinv(parent_, T_local_);
      }
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv(cnoid::Link* parent, cnoid::Position& T_local);//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる

      virtual bool isValid() {return parent_;}

    protected:
      std::string name_;

      cnoid::Link* parent_;
      cnoid::Position T_local_;
      cnoid::Vector6 F_;

      // cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> Rinv_;
      cnoid::JointPath path_;

    };

  };
};

#endif
