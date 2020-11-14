#ifndef CONTACT_FORCE_ESTIMATOR_H
#define CONTACT_FORCE_ESTIMATOR_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <memory>
#include <Eigen/Sparse>
#include <prioritized_qp/PrioritizedQPSolver.h>

namespace multicontact_controller {

  class ContactPoint {
  public:
    std::string name() const { return name_; }
    std::string& name() { return name_; }

    cnoid::Link* const parent() const { return parent_; }
    cnoid::Link*& parent() { return parent_; }

    cnoid::Position T_local() const { return T_local_; }
    cnoid::Position& T_local() {return T_local_; }

    cnoid::Vector6 F() const { return F_; }
    cnoid::Vector6& F() {return F_; }

    // ContactForceEstimatorが内部で使う
    cnoid::Link* const parent_est() const { return parent_est_; }
    cnoid::Link*& parent_est() { return parent_est_; }

    cnoid::Position T_local_est() const { return T_local_est_; }
    cnoid::Position& T_local_est() {return T_local_est_; }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian();//world系,contactpoint周り
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
  private:
    std::string name_;

    cnoid::Link* parent_;
    cnoid::Position T_local_;

    cnoid::Vector6 F_;

    cnoid::Link* parent_est_;
    cnoid::Position T_local_est_;

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> Rinv_;
    cnoid::JointPath path_;

  };

  class ContactForceEstimator {
  public:
    // robotのFKと関節トルク、力センサの値はユーザーが入力すること
    // robotのマスパラはsetRobotする前に編集すること
    bool setRobot(const cnoid::Body* robot);

    bool setCandidatePoint(std::shared_ptr<ContactPoint> candidatepoint);
    std::shared_ptr<ContactPoint> getCandidatePoint(const std::string& name);
    bool deleteCandidatePoint(const std::string& name);
    bool clearCandidatePoints();

    bool estimateForce();

    cnoid::Vector6 getEstimatedForce(const std::string& name);

    bool removeForceSensorOffset(double time);
    bool remoteRootForceOffset(double time);
    bool remoteJointTorqueOffset(double time);

  protected:
    bool updateRobotState();

    const cnoid::Body* robot_org_;
    std::shared_ptr<cnoid::Body> robot_;

    std::vector<std::shared_ptr<ContactPoint> > candidatePoints_;
    bool changed_ = true;

    // cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    std::vector<std::shared_ptr<prioritized_qp::Task> > tasks_;
  };

};

#endif
