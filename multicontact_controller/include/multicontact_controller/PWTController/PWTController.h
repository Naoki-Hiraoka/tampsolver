#ifndef PWT_CONTROLLER_H
#define PWT_CONTROLLER_H

#include <cnoid/Body>
#include <memory>
#include <Eigen/Sparse>
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>
#include <prioritized_qp/PrioritizedQPSolver.h>

namespace multicontact_controller {
  class ContactPointPWTC: public cnoidbodyutils::ContactPoint {
  public:
    // S J dqa = 0 となるSを返す. ? x 6
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& selectMatrixForKinematicsConstraint();
    // KinematicsConstraint による拘束力の接触維持に必要な制約を返す.
    // /iterの次元
    void contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);
    // KinematicsConstraint による拘束力の目標値を返す。主に接触解除時用
    // /iterの次元
    void desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);
    // KinematicsConstraint による拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
    // 無次元
    void bestEffortForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    // 位置の目標値を返す。主に遊脚用. colは[root6dof + numJoint]
    // rad m / iter
    void desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A_A, cnoid::VectorX& b_A, cnoid::VectorX& wa_A, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_A, cnoid::VectorX& dl_A, cnoid::VectorXd& du_A, cnoid::VectorX& wc_A,
                                   Eigen::SparseMatrix<double,Eigen::RowMajor>& A_B, cnoid::VectorX& b_B, cnoid::VectorX& wa_B, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_B, cnoid::VectorX& dl_B, cnoid::VectorXd& du_B, cnoid::VectorX& wc_B);

    void update(const cnoid::VectorX& dqa){
      cnoid::Vector6 vel = this->calcRinv() * this->calcJacobian() * dqa; //local系
      this->interaction_->update(vel.head<3>(), vel.tail<3>());
    }

    std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    bool isValid() override {
      return cnoidbodyutils::ContactPoint::isValid() && contact_ && interaction_;
    }

    std::shared_ptr<cnoidbodyutils::Contact> contact() const { return contact_;}
    std::shared_ptr<cnoidbodyutils::Contact>& contact() { return contact_;}
    std::shared_ptr<cnoidbodyutils::Interaction> interaction() const { return interaction_;}
    std::shared_ptr<cnoidbodyutils::Interaction>& interaction() { return interaction_;}

    std::string& state() { return state_;}
    std::string state() const { return state_;}
    double& k() { return k_;}
    double k() const { return k_;}
    double& dt() { return dt_;}
    double dt() const { return dt_;}
  private:

    std::shared_ptr<cnoidbodyutils::Contact> contact_;
    std::shared_ptr<cnoidbodyutils::Interaction> interaction_;

    std::string state_;
    double k_;
    double dt_;

    //cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> selectMatrixForKinematicsConstraint_;
  };

  class PWTController {
  public:
    PWTController(cnoid::Body* robot,
                  std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos // jointIdの順に並んでいる必要がある
                  )
      : robot_(robot),
        jointInfos_(jointInfos),
        torqueJacobianCalculator_(robot_),
        //params
        debug_print_(false),
        sv_ratio_(1e-12),
        k0_(0.1),
        tolerance0_1_(0.02),
        k0_1_(0.5),
        w0_1_(1e-2),
        we0_1_(1e6),
        k1_(5.0),
        w1_(1e-2),
        we1_(1e4),
        w_scale1_(4e0),
        tau_scale1_(4e0),
        tolerance1_1_(0.04),
        k1_1_(0.5),
        w1_1_(1e-2),
        we1_1_(1e6),
        w2_(1e-2),
        we2_(1e4),
        w2_5_(1e-2),
        we2_5_(1e4),
        w_scale2_5_(4e0),
        k3_(5.0),
        w3_(1e1),
        w_scale3_(1e1),
        tau_scale3_(1e1),
        w_weight3_(1e-1),
        tau_weight3_(1e0),
        taumax_weight3_(1e1),

        //cache
        Ka_(6+robot_->numJoints(),6+robot_->numJoints())
    {
    }

    //calcForwardKinematics()とcalcCM()が事前に必要
    bool calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                        std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                        std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& pclCollisions,
                        double dt);

    bool debug_print() const {return debug_print_;}
    bool& debug_print() {return debug_print_;}
    double sv_ratio() const { return sv_ratio_;}
    double& sv_ratio() { return sv_ratio_;}
    double k0() const { return k0_;}
    double& k0() { return k0_;}
    double tolerance0_1() const { return tolerance0_1_;}
    double& tolerance0_1() { return tolerance0_1_;}
    double k0_1() const { return k0_1_;}
    double& k0_1() { return k0_1_;}
    double w0_1() const { return w0_1_;}
    double& w0_1() { return w0_1_;}
    double we0_1() const { return we0_1_;}
    double& we0_1() { return we0_1_;}
    double k1() const { return k1_;}
    double& k1() { return k1_;}
    double w1() const { return w1_;}
    double& w1() { return w1_;}
    double we1() const { return we1_;}
    double& we1() { return we1_;}
    double w_scale1() const { return w_scale1_;}
    double& w_scale1() { return w_scale1_;}
    double tau_scale1() const { return tau_scale1_;}
    double& tau_scale1() { return tau_scale1_;}
    double tolerance1_1() const { return tolerance1_1_;}
    double& tolerance1_1() { return tolerance1_1_;}
    double k1_1() const { return k1_1_;}
    double& k1_1() { return k1_1_;}
    double w1_1() const { return w1_1_;}
    double& w1_1() { return w1_1_;}
    double we1_1() const { return we1_1_;}
    double& we1_1() { return we1_1_;}
    double w2() const { return w2_;}
    double& w2() { return w2_;}
    double we2() const { return we2_;}
    double& we2() { return we2_;}
    double w2_5() const { return w2_5_;}
    double& w2_5() { return w2_5_;}
    double we2_5() const { return we2_5_;}
    double& we2_5() { return we2_5_;}
    double w_scale2_5() const { return w_scale2_5_;}
    double& w_scale2_5() { return w_scale2_5_;}
    double k3() const { return k3_;}
    double& k3() { return k3_;}
    double w3() const { return w3_;}
    double& w3() { return w3_;}
    double w_scale3() const { return w_scale3_;}
    double& w_scale3() { return w_scale3_;}
    double tau_scale3() const { return tau_scale3_;}
    double& tau_scale3() { return tau_scale3_;}
    double w_weight3() const { return w_weight3_;}
    double& w_weight3() { return w_weight3_;}
    double tau_weight3() const { return tau_weight3_;}
    double& tau_weight3() { return tau_weight3_;}
    double taumax_weight3() const { return taumax_weight3_;}
    double& taumax_weight3() { return taumax_weight3_;}
  protected:
    // メンバ変数はKa_しか使わない
    bool calcPWTJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,//返り値
                         std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,//返り値
                         Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,//返り値
                         cnoid::Body* robot,
                         std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                         cnoidbodyutils::TorqueJacobianCalculator& torqueJacobianCalculator,
                         std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                         std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Js,
                         double sv_ratio);

    // 各行はm/iter, rad/iterのオーダーにそろえる

    // メンバ変数はdebug_print_とTask0_しか使わない
    bool setupTask0(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                    double k,
                    double dt);

    // メンバ変数はdebug_print_とTask0_1_しか使わない
    bool setupTask0_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                      cnoid::Body* robot,
                      std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                      std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                      double tolerance,
                      double k,
                      double dt,
                      double w,
                      double we);

    // メンバ変数はdebug_print_とTask1_しか使わない
    bool setupTask1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                    double w_scale,//次元の大きさを揃え、計算を安定化する
                    double tau_scale,//次元の大きさを揃え、計算を安定化する
                    double k,
                    double dt,
                    double w,
                    double we);

    // メンバ変数はdebug_print_とTask1_1_しか使わない
    bool setupTask1_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                      cnoid::Body* robot,
                      std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                      std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& pclCollisions,
                      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                      double tolerance,
                      double k,
                      double dt,
                      double w,
                      double we);

    // メンバ変数はdebug_print_とTask2_A_, task2_B_しか使わない
    bool setupTask2(std::shared_ptr<prioritized_qp::Task>& taskA, //返り値
                    std::shared_ptr<prioritized_qp::Task>& taskB, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                    double dt,
                    double w,
                    double we);

    // メンバ変数はdebug_print_とTask2_5_しか使わない
    bool setupTask2_5(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                      cnoid::Body* robot,
                      std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                      std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                      const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                      double w_scale,//次元の大きさを揃え、計算を安定化する
                      double dt,
                      double w,
                      double we);

    // メンバ変数はdebug_print_とTask3Helper_しか使わない
    bool setupTask3(std::shared_ptr<prioritized_qp::Task>& taskHelper, //返り値,
                    std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                    double w_scale,//次元の大きさを揃え、計算を安定化する
                    double tau_scale,//次元の大きさを揃え、計算を安定化する
                    double w_weight,
                    double tau_weight,
                    double taumax_weight,//tauに比したtaumaxの重み
                    double k,
                    double dt,
                    double w);

  private:
    cnoid::Body* robot_;
    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos_;
    cnoidbodyutils::TorqueJacobianCalculator torqueJacobianCalculator_;

    // params
    bool debug_print_;
    //   calcPWTJacobian
    double sv_ratio_;
    //   setupTask0
    double k0_;
    //   setupTask0_1
    double tolerance0_1_;
    double k0_1_;
    double w0_1_;
    double we0_1_;
    //   setupTask1
    double k1_;
    double w1_;
    double we1_;
    double w_scale1_;
    double tau_scale1_;
    //   setupTask1_1
    double tolerance1_1_;
    double k1_1_;
    double w1_1_;
    double we1_1_;
    //   setupTask2
    double w2_;
    double we2_;
    //   setupTask2_5
    double w2_5_;
    double we2_5_;
    double w_scale2_5_;
    //   setupTask3
    double k3_;
    double w3_;
    double w_scale3_;
    double tau_scale3_;
    double w_weight3_;
    double tau_weight3_;
    double taumax_weight3_;

    // cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> Ka_; //calcPWTJacobian
    std::shared_ptr<prioritized_qp::Task> task0_; // setupTask0
    std::shared_ptr<prioritized_qp::Task> task0_1_; // setupTask0_1
    std::shared_ptr<prioritized_qp::Task> task1_; // setupTask1
    std::shared_ptr<prioritized_qp::Task> task1_1_; // setupTask1_1
    std::shared_ptr<prioritized_qp::Task> task2_A_; // setupTask2
    std::shared_ptr<prioritized_qp::Task> task2_B_; // setupTask2
    std::shared_ptr<prioritized_qp::Task> task2_5_; // setupTas2_5
    std::shared_ptr<prioritized_qp::Task> task3Helper_; // setupTask3_Helper
    std::shared_ptr<prioritized_qp::Task> task3_; // setupTask3
  };

};

#endif
