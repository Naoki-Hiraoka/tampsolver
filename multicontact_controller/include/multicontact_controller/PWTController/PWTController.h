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
    void contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);
    // KinematicsConstraint による拘束力の目標値を返す。主に接触解除時用
    void desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);
    // KinematicsConstraint による拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
    void bestEffortForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    // 位置の目標値を返す。主に遊脚->支持脚遷移用. colは[root6dof + numJoint]
    void makeContactPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    // 位置の目標値を返す。主に遊脚用. colは[root6dof + numJoint]
    void desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    void update(const cnoid::VectorX& dqa){
      cnoid::Vector6 vel = this->calcJacobian() * dqa;
      this->interaction_->update(vel.head<3>(), vel.tail<3>());
    }

    std::shared_ptr<cnoidbodyutils::Contact> contact() const { return contact_;}
    std::shared_ptr<cnoidbodyutils::Contact>& contact() { return contact_;}
    std::shared_ptr<cnoidbodyutils::Interaction> interaction() const { return interaction_;}
    std::shared_ptr<cnoidbodyutils::Interaction>& interaction() { return interaction_;}

    std::string& state() { return state_;}
    std::string state() const { return state_;}
  private:

    std::shared_ptr<cnoidbodyutils::Contact> contact_;
    std::shared_ptr<cnoidbodyutils::Interaction> interaction_;

    std::string state_;

    //cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> selectMatrixForKinematicsConstraint_;
  };

  class JointInfo {
  public:
    JointInfo();

    // 指令関節角度上下限に関する制約を返す.破損防止
    void JointAngleConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);

    // M \tau によってこのJointの成分を抽出できるM(select matrix). \tauは[numJoints]
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& torqueSelectMatrix();

    // 関節トルク上下限に関する制約を返す.破損防止
    void JointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);
    // 関節トルクのベストエフォートタスクを返す。主に負荷低減用
    void bestEffortJointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

    std::string& name() { return name_;}
    std::string name() const { return name_;}
    cnoid::Link*& joint() { return joint_;}
    cnoid::Link* joint() const { return joint_;}

    // 指令関節角度を変えてよいか
    bool& controllable() { return controllable_;}
    bool controllable() const { return controllable_;}
    // 関節トルク低減を考慮するか
    bool& care_torque() { return care_torque_;}
    bool care_torque() const { return care_torque_;}
    double& command_angle() {return command_angle_;}
    double command_angle() const {return command_angle_;}
    double dt() const { return dt_;}
    double& dt() { return dt_;}

    double& coil_temperature_limit() { return coil_temperature_limit_;}
    double coil_temperature_limit() const { return coil_temperature_limit_;}
    double& housing_temperature() { return housing_temperature_;}
    double housing_temperature() const { return housing_temperature_;}
    double& coil_temperature() { return coil_temperature_;}
    double coil_temperature() const { return coil_temperature_;}
    double& maximum_effort_soft() { return maximum_effort_soft_;}
    double maximum_effort_soft() const { return maximum_effort_soft_;}
    double& maximum_effort_hard() { return maximum_effort_hard_;}
    double maximum_effort_hard() const { return maximum_effort_hard_;}
    double& balance_effort() { return balance_effort_;}
    double balance_effort() const { return balance_effort_;}
    double& remaining_time() { return remaining_time_;}
    double remaining_time() const { return remaining_time_;}

    double& pgain() { return pgain_;}
    double pgain() const { return pgain_;}
    double& dgain() { return dgain_;}
    double dgain() const { return dgain_;}

    std::shared_ptr<cnoidbodyutils::JointLimitTable>& jointLimitTable() { return jointLimitTable_;}
    std::shared_ptr<cnoidbodyutils::JointLimitTable> jointLimitTable() const { return jointLimitTable_;}
    std::weak_ptr<JointInfo> jointLimitTableTargetJointInfo() const { return jointLimitTableTargetJointInfo_;}
    std::weak_ptr<JointInfo>& jointLimitTableTargetJointInfo() { return jointLimitTableTargetJointInfo_;}
  protected:
    std::string name_;
    cnoid::Link* joint_;

    bool controllable_;
    bool care_torque_;
    double command_angle_;
    double dt_;

    double coil_temperature_limit_;
    double housing_temperature_;
    double coil_temperature_;
    double maximum_effort_soft_;
    double maximum_effort_hard_;
    double balance_effort_;
    double remaining_time_;

    double pgain_;
    double dgain_;

    double ulimit_;
    double llimit_;
    double uvlimit_;
    double lvlimit_;

    std::shared_ptr<cnoidbodyutils::JointLimitTable> jointLimitTable_;
    std::weak_ptr<JointInfo> jointLimitTableTargetJointInfo_;

    //cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> torqueSelectMatrix_;
  };

  class PWTController {
  public:
    PWTController(cnoid::Body* robot,
                  std::vector<std::shared_ptr<JointInfo> >& jointInfos // jointIdの順に並んでいる必要がある
                  )
      : robot_(robot),
        jointInfos_(jointInfos),
        torqueJacobianCalculator_(robot_),
        //params
        sv_ratio_(1e-12),
        k0_(0.1),
        k1_(5.0),
        w1_(1e-2),
        we1_(1e-8),
        w_scale1_(1e6),//hessianが大きい気がする
        tau_scale1_(1e6),
        w2_(1e-2),
        we2_(1e4),
        k3_(5.0),//TODO
        w3_(1e-2),//TODO
        w_scale3_(1e6),//TODO
        tau_scale3_(1e6),//TODO
        taumax_weight3_(1e2),//TODO

        //cache
        Ka_(6+robot_->numJoints(),6+robot_->numJoints())
    {
    }

    //calcForwardKinematics()とcalcCM()が事前に必要
    bool calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints, double dt);

    double& sv_ratio() { return sv_ratio_;}
    double sv_ratio() const { return sv_ratio_;}

    double& k0() { return k0_;}
    double k0() const { return k0_;}

  protected:
    // メンバ変数はKa_しか使わない
    bool calcPWTJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,//返り値
                         std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,//返り値
                         Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,//返り値
                         cnoid::Body* robot,
                         std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                         cnoidbodyutils::TorqueJacobianCalculator& torqueJacobianCalculator,
                         std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                         std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > >& Js,
                         double sv_ratio);

    // メンバ変数はTask0_しか使わない
    bool setupTask0(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                    double k,
                    double dt,
                    size_t additional_cols_before = 0,
                    size_t additional_cols_after = 0);

    // メンバ変数はTask1_しか使わない
    bool setupTask1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                    double w_scale,//次元の大きさを揃え、計算を安定化する
                    double tau_scale,//次元の大きさを揃え、計算を安定化する
                    double k,
                    double dt,
                    double w,
                    double we,
                    size_t additional_cols_before = 0,
                    size_t additional_cols_after = 0);

    // メンバ変数はTask2_しか使わない
    bool setupTask2(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                    double dt,
                    double w,
                    double we,
                    size_t additional_cols_before = 0,
                    size_t additional_cols_after = 0);

    // メンバ変数はTask3_しか使わない
    bool setupTask3(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                    cnoid::Body* robot,
                    std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                    std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                    const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                    double w_scale,//次元の大きさを揃え、計算を安定化する
                    double tau_scale,//次元の大きさを揃え、計算を安定化する
                    double taumax_weight,//tauに比したtaumaxの重み
                    double k,
                    double dt,
                    double w,
                    size_t additional_cols_before = 0,
                    size_t additional_cols_after = 0);

    double dampingFactor(double w,
                         double we,
                         const cnoid::VectorX& b,
                         const cnoid::VectorX& wa,
                         const cnoid::VectorX& dl,
                         const cnoid::VectorX& du,
                         const cnoid::VectorX& wc);
  private:
    cnoid::Body* robot_;
    std::vector<std::shared_ptr<JointInfo> >& jointInfos_;
    cnoidbodyutils::TorqueJacobianCalculator torqueJacobianCalculator_;

    // params
    //   calcPWTJacobian
    double sv_ratio_;
    //   setupTask0
    double k0_;
    //   setupTask1
    double k1_;
    double w1_;
    double we1_;
    double w_scale1_;
    double tau_scale1_;
    //   setupTask2
    double w2_;
    double we2_;
    //   setupTask3
    double k3_;
    double w3_;
    double w_scale3_;
    double tau_scale3_;
    double taumax_weight3_;

    // cache
    Eigen::SparseMatrix<double,Eigen::RowMajor> Ka_; //calcPWTJacobian
    std::vector<std::shared_ptr<prioritized_qp::Task> > tasks_; // calcPWTControl
    std::shared_ptr<prioritized_qp::Task> task0_; // setupTask0
    std::shared_ptr<prioritized_qp::Task> task1_; // setupTask1
    std::shared_ptr<prioritized_qp::Task> task2_; // setupTask2
    std::shared_ptr<prioritized_qp::Task> task3_; // setupTask3
  };

};

#endif
