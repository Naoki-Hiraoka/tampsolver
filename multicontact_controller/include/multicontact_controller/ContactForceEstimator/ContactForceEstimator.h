#ifndef CONTACT_FORCE_ESTIMATOR_H
#define CONTACT_FORCE_ESTIMATOR_H

#include <cnoid/Body>
#include <memory>
#include <Eigen/Sparse>
#include <prioritized_qp/PrioritizedQPSolver.h>

#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {

  class ContactPointCFE: public cnoidbodyutils::ContactPoint {
  public:

    // ContactForceEstimatorが内部で使う
    cnoid::Link* const parent_est() const { return parent_est_; }
    cnoid::Link*& parent_est() { return parent_est_; }

    cnoid::Position T_local_est() const { return T_local_est_; }
    cnoid::Position& T_local_est() {return T_local_est_; }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcJacobian() override {
      return cnoidbodyutils::ContactPoint::calcJacobian(parent_est_, T_local_est_);
    }
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcRinv() override {
      return cnoidbodyutils::ContactPoint::calcRinv(parent_est_, T_local_est_);
    }
  private:

    cnoid::Link* parent_est_;
    cnoid::Position T_local_est_;

  };

  class ContactForceEstimator {
  public:
    // robotのFKと関節トルク、力センサの値はユーザーが入力すること
    // robotのマスパラはsetRobotする前に編集すること
    bool setRobot(const cnoid::Body* robot);

    bool setCandidatePoint(std::shared_ptr<ContactPointCFE> candidatepoint);
    std::shared_ptr<ContactPointCFE> getCandidatePoint(const std::string& name);
    bool deleteCandidatePoint(const std::string& name);
    bool clearCandidatePoints();

    bool estimateForce();

    cnoid::Vector6 getEstimatedForce(const std::string& name);
    cnoid::Vector6 getOffsetForce(const std::string& name);

    cnoid::Vector6 getRootForceOffset(){
      return rootForceOffset_;
    }

  protected:
    bool updateRobotState();

    const cnoid::Body* robot_org_;
    std::shared_ptr<cnoid::Body> robot_;

    std::vector<std::shared_ptr<ContactPointCFE> > candidatePoints_;
    bool changed_ = true;

    std::vector<cnoid::Vector6> forceSensorOffsets_;
    cnoid::Vector6 rootForceOffset_;

    // cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian_;
    std::vector<std::shared_ptr<prioritized_qp::Task> > tasks_;
  };

};

#endif
