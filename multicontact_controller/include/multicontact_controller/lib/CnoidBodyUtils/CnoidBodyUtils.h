#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Sparse>

#include <multicontact_controller/lib/CnoidBodyUtils/Drawing.h>
#include <multicontact_controller/lib/CnoidBodyUtils/Contact.h>
#include <multicontact_controller/lib/CnoidBodyUtils/ContactPoint.h>
#include <multicontact_controller/lib/CnoidBodyUtils/TorqueJacobianCalculator.h>
#include <multicontact_controller/lib/CnoidBodyUtils/JointLimitTable.h>
#include <multicontact_controller/lib/CnoidBodyUtils/Interaction.h>
#include <multicontact_controller/lib/CnoidBodyUtils/Collision.h>
#include <multicontact_controller/lib/CnoidBodyUtils/JointInfo.h>

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
    bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor>& Mout);
    bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor>& Mout);

    double dampingFactor(double w,
                         double we,
                         const cnoid::VectorX& b,
                         const cnoid::VectorX& wa,
                         const cnoid::VectorX& dl,
                         const cnoid::VectorX& du,
                         const cnoid::VectorX& wc);

    // colは[rootlink, numJoints]. rootlinkはworld系, rootlinkまわり. rowはworld系
    void calcCMJacobian(cnoid::Body* robot, Eigen::SparseMatrix<double, Eigen::RowMajor>& CMJ);

    // extに、最大値の増減を表す変数が設定される. Asが空なら、maximumは負の値になりうる
    bool defineMaximumError(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& As,
                            const std::vector<Eigen::VectorXd>& bs,
                            const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Cs,
                            const std::vector<Eigen::VectorXd>& dls,
                            const std::vector<Eigen::VectorXd>& dus,
                            std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& CsHelper,
                            std::vector<Eigen::VectorXd>& dlsHelper,
                            std::vector<Eigen::VectorXd>& dusHelper,
                            std::vector<Eigen::VectorXd>& wcsHelper,
                            std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& C_extsHelper,
                            double& maximum,
                            int As_begin_idx = 0,//Asのいくつ目からを見るか
                            int As_end_idx = -1,//Asのいくつ目までを見るか. 負なら最後まで
                            int Cs_begin_idx = 0,//Csのいくつ目からを見るか
                            int Cs_end_idx = -1//Csのいくつ目までを見るか. 負なら最後まで
                            );

    bool copyBodyKinematicsState(const cnoid::Body* robot_in, cnoid::Body* robot_out);
  };
};

#endif
