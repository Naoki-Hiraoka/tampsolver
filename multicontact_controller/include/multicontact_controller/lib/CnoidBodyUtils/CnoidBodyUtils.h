#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Sparse>

#include <multicontact_controller/lib/CnoidBodyUtils/Contact.h>
#include <multicontact_controller/lib/CnoidBodyUtils/ContactPoint.h>
#include <multicontact_controller/lib/CnoidBodyUtils/TorqueJacobianCalculator.h>
#include <multicontact_controller/lib/CnoidBodyUtils/JointLimitTable.h>
#include <multicontact_controller/lib/CnoidBodyUtils/Interaction.h>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    // rosparamのファイル名からロードする
    cnoid::Body* loadBodyFromParam(const std::string& paramname);

    //choreonoidのロボットモデルはリンク名が関節名によって管理されている
    cnoid::Link* getLinkFromURDFlinkName(cnoid::Body* robot, const std::string& linkname);

    // ROS topic to Body
    void jointStateToBody(const sensor_msgs::JointState::ConstPtr& msg, cnoid::Body* robot);

    void imuToBody(const sensor_msgs::Imu::ConstPtr& msg, cnoid::Body* robot);

    void odomToBody(const nav_msgs::Odometry::ConstPtr& msg, cnoid::Body* robot);

    // EusLispと同じ
    cnoid::Matrix3 orientCoordsToAxis(const cnoid::Matrix3& coords, const cnoid::Vector3& axis/*local 系*/ = cnoid::Vector3::UnitZ(), const cnoid::Vector3& target_axis/*world系*/ = cnoid::Vector3::UnitZ());

    size_t calcPseudoInverse(const cnoid::MatrixXd &M, cnoid::MatrixXd &Minv, double sv_ratio=1.0e-3);

    bool appendRow(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor>& Mout);
    bool appendRow(const std::vector<cnoid::VectorX>& vs, cnoid::VectorX& vout);

  };
};

#endif
