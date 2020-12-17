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

    // EusLispと同じ
    cnoid::Matrix3 orientCoordsToAxis(const cnoid::Matrix3& coords, const cnoid::Vector3& axis/*local 系*/ = cnoid::Vector3::UnitZ(), const cnoid::Vector3& target_axis/*world系*/ = cnoid::Vector3::UnitZ());

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

    class TorqueJacobianCalculator {
    public:
      TorqueJacobianCalculator(cnoid::Body* robot);

      // いわゆるトルクヤコビアン. [root6dof + numJoints] x [root6dof + numJoints]. world原点がrootの位置にあるとみなして、root6dofは原点まわりの力の釣り合いを考える. そのため、root6dofはrootの位置、world系で、各軸独立(シリアルでない, 位置と回転軸が変位前後で不変).
      // 事前にcalcForWardKinematics(false,false)とcalcCM()が必要
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcDg(const cnoid::Vector3 g = cnoid::Vector3(0, 0, 9.80665));
      // 事前にcalcForWardKinematics(false,false)が必要。contactPointsのFを用いる.FはContactPointに固定されており、worldに固定されているのではない
      const Eigen::SparseMatrix<double,Eigen::RowMajor>&  calcDJw(std::vector<std::shared_ptr<ContactPoint> > contactPoints);

      // 最初に一回呼ばれる
      bool generateRelationMap();

      // calcDg中に一回呼ばれる
      // copied from Choreonoid/Body/Jacobian.cpp
      struct SubMass{
        double m;
        cnoid::Vector3 mwc;
      };
      void calcSubMass(cnoid::Link* link);

      enum class Relation { OTHER_PATH, SAME_JOINT, ANCESTOR, DESCENDANT };

    protected:
      // relationMap[i][j]: iがjの何か.
      std::map<cnoid::Link*,std::map<cnoid::Link*, Relation> > relationMap_;
      cnoid::Body* robot_;

      // utility. rootjointの軸
      std::vector<cnoid::Vector3> rootAxis_;

      // cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> Dg_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> DJw_;
      std::vector<SubMass> subMasses_;
    };

  };
};

#endif
