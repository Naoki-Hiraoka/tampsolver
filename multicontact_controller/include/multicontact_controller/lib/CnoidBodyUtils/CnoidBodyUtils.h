#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <Eigen/Sparse>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    // rosparamのファイル名からロードする
    cnoid::Body* loadBodyFromParam(const std::string& paramname);

    //choreonoidのロボットモデルはリンク名が関節名によって管理されている
    cnoid::Link* getLinkFromURDFlinkName(cnoid::Body* robot, const std::string& linkname);

    // ROS topic to Body
    void jointStateToBody(const sensor_msgs::JointState::ConstPtr& msg, cnoid::Body* robot);

    void imuToBody(const sensor_msgs::Imu::ConstPtr& msg, cnoid::Body* robot);

    //
    cnoid::Matrix3 orientCoordsToAxis(const cnoid::Matrix3& coords, const cnoid::Vector3& axis/*local 系*/ = cnoid::Vector3::UnitZ(), const cnoid::Vector3& target_axis/*world系*/ = cnoid::Vector3::UnitZ());

    class ContactPoint {
    public:
      std::string name() const { return name_; }
      std::string& name() { return name_; }

      cnoid::Link* const parent() const { return parent_; }
      cnoid::Link*& parent() { return parent_; }

      cnoid::Position T_local() const { return T_local_; }
      cnoid::Position& T_local() {return T_local_; }

      // これらの関数はクラスから分離してもよいが、配列などをキャッシュしたいので
      virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian() {
        return calcJacobian(parent_, T_local_);
      }
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian(cnoid::Link* parent, cnoid::Position& T_local);//world系,contactpoint周り
      virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv() {
        return calcRinv(parent_, T_local_);
      }
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv(cnoid::Link* parent, cnoid::Position& T_local);//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる

    protected:
      std::string name_;

      cnoid::Link* parent_;
      cnoid::Position T_local_;

      // cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> Rinv_;
      cnoid::JointPath path_;

    };
  };
};

#endif
