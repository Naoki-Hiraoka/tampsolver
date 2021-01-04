#ifndef CONTACT_BREAKABILITY_CHECKER_H
#define CONTACT_BREAKABILITY_CHECKER_H

#include <cnoid/Body>
#include <cnoid/SceneMarkers>
#include <memory>
#include <Eigen/Sparse>
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>
#include <prioritized_qp/PrioritizedQPSolver.h>
#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROS.h>

namespace multicontact_controller {
  class ContactPointCBAC: public cnoidbodyutils::ContactPoint {
  public:
    // S J dqa = 0 となるSを返す. ? x 6
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& selectMatrixForKinematicsConstraint();
    // KinematicsConstraint による拘束力の接触維持に必要な制約を返す.
    // 各行は無次元化されている. selectMatrixForKinematicsConstraintの次元
    void contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);
    // KinematicsConstraint による拘束力の目標値を返す。主に接触解除時用
    // 各行は無次元化されている. selectMatrixForKinematicsConstraintの次元
    void desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    // 支持脚の場合位置の目標値を返す。 colは[root6dof + numJoint]
    // T_actが必要
    // rad m / iter
    void desiredPositionConstraintBal(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);
    // 遊脚の場合位置の目標値を返す。 colは[root6dof + numJoint]
    // T_actが必要
    // rad m / iter
    void desiredPositionConstraintInt(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    std::shared_ptr<cnoidbodyutils::Contact> contact() const { return contact_;}
    std::shared_ptr<cnoidbodyutils::Contact>& contact() { return contact_;}

    cnoid::Position& T_act() { return T_act_;}
    cnoid::Position T_act() const { return T_act_;}
    std::string& state() { return state_;}
    std::string state() const { return state_;}
  private:
    std::shared_ptr<cnoidbodyutils::Contact> contact_;
    cnoid::Position T_act_;
    std::string state_;

    // visualize
    cnoid::SgLineSetPtr lines_;

    //cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> selectMatrixForKinematicsConstraint_;
  };

  class ContactBreakAbilityChecker {
  public:
    ContactBreakAbilityChecker(cnoid::Body* robot,
                               std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos // jointIdの順に並んでいる必要がある
                               )
      : robot_(robot),
        jointInfos_(jointInfos),
        loopNum_(30),
        debug_print_(false)
    {
      robot_->calcCenterOfMass();//mass()を有効にする
    }

    // 現在のcontactPointsの状態でbreakContactPointのcontactをbreakできるかどうかを調べ、重心位置の余裕を返す
    bool check(double& margin, //返り値
               std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
               std::shared_ptr<ContactPointCBAC>& breakContactPoint,
               bool fixInteraction,//interaction eefを固定するか
               std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
               std::shared_ptr<PCLCollisionDetector>& pCLCollisionDetector
               );

    void setIKLoopCb(std::function<void()> f) {ikLoopCb_ = f; }
    std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    // IKを終了する回数
    size_t loopNum() const { return loopNum_;}
    size_t& loopNum() { return loopNum_;}
  protected:
    // メンバ変数はrobot_しか使わない
    bool calcSCFR(std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
                  const std::shared_ptr<ContactPointCBAC>& breakContactPoint,
                  Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,//返り値
                  cnoid::VectorXd& SCFR_l,//返り値
                  cnoid::VectorX& SCFR_u,//返り値
                  std::vector<cnoid::Vector2>& vertices,
                  double g = 9.80665
                  );

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupJointLimitTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                             cnoid::Body* robot,
                             std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                             double w_);

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupSelfCollisionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                cnoid::Body* robot,
                                std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                double tolerance,
                                double w,
                                double we);

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupPositionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                           cnoid::Body* robot,
                           std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                           std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
                           bool fixInteraction,
                           double w,
                           double we);

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupSCFRTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                       cnoid::Body* robot,
                       std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                       const Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,
                       const cnoid::VectorXd& SCFR_l,
                       const cnoid::VectorX& SCFR_u,
                       const cnoid::Vector3& initialCenterOfMass,
                       double w,
                       double we);

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupPCLCollisionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                               cnoid::Body* robot,
                               std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                               std::shared_ptr<PCLCollisionDetector>& pCLCollisionDetector,
                               double tolerance,
                               double w,
                               double we);

    // メンバ変数はdebug_print_とTask_しか使わない
    bool setupSCFRBreakTask(std::shared_ptr<prioritized_qp::Task>& taskHelper, //返り値
                            std::shared_ptr<prioritized_qp::Task>& task, //返り値
                            cnoid::Body* robot,
                            std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                            const Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,
                            const cnoid::VectorXd& SCFR_l,
                            const cnoid::VectorX& SCFR_u,
                            const cnoid::Vector3& initialCenterOfMass,
                            double w);

  private:

    cnoid::Body* robot_;
    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> > jointInfos_;

    // for visualize
    std::function<void()> ikLoopCb_;

    // param
    size_t loopNum_;
    bool debug_print_;
    double w_JointLimit_;
    double tolerance_SelfCollision_;
    double w_SelfCollision_;
    double we_SelfCollision_;
    double w_Position_;
    double we_Position_;
    double w_SCFR_;
    double we_SCFR_;
    double tolerance_PCLCollision_;
    double w_PCLCollision_;
    double we_PCLCollision_;
    double w_SCFR_break_;

    //cache
    std::shared_ptr<prioritized_qp::Task> jointLimitTask_;
    std::shared_ptr<prioritized_qp::Task> selfCollisionTask_;
    std::shared_ptr<prioritized_qp::Task> positionTask_;
    std::shared_ptr<prioritized_qp::Task> sCFRTask_;
    std::shared_ptr<prioritized_qp::Task> pCLCollisionTask_;
    std::shared_ptr<prioritized_qp::Task> sCFRBreakTaskHelper_;
    std::shared_ptr<prioritized_qp::Task> sCFRBreakTask_;

    // visualize
    cnoid::Vector3 sCFRCenterOfMass_;
    std::vector<cnoid::Vector2> sCFRVertices_;
    std::vector<cnoid::Vector2> sCFRBreakVertices_;
    cnoid::SgLineSetPtr sCFRLines_;
    cnoid::SgLineSetPtr sCFRBreakLines_;
    cnoid::CrossMarkerPtr comMarker_;
  };

};

#endif
