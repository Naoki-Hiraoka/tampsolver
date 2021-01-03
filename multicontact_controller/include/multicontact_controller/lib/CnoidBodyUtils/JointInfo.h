#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_JOINTINFO_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_JOINTINFO_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>
#include <multicontact_controller/lib/CnoidBodyUtils/JointLimitTable.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    class JointInfo {
    public:
      JointInfo();

      // 指令関節角度上下限に関する制約を返す.破損防止
      void JointAngleConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc, bool use_command_angle = false);

      // 指令関節角速度上下限に関する制約を返す.破損防止
      void JointVelocityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc);

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

      double& llimit() { return llimit_;}
      double llimit() const { return llimit_;}
      double& ulimit() { return ulimit_;}
      double ulimit() const { return ulimit_;}
      double& lvlimit() { return lvlimit_;}
      double lvlimit() const { return lvlimit_;}
      double& uvlimit() { return uvlimit_;}
      double uvlimit() const { return uvlimit_;}

      std::shared_ptr<JointLimitTable>& jointLimitTable() { return jointLimitTable_;}
      std::shared_ptr<JointLimitTable> jointLimitTable() const { return jointLimitTable_;}
      std::weak_ptr<JointInfo> jointLimitTableTargetJointInfo() const { return jointLimitTableTargetJointInfo_;}
      std::weak_ptr<JointInfo>& jointLimitTableTargetJointInfo() { return jointLimitTableTargetJointInfo_;}

      // ここから下は、setupJointInfoFromParamではセットされない
      //  motor_temperature_state
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
      //  motor_state
      double& pgain() { return pgain_;}
      double pgain() const { return pgain_;}
      double& hardware_pgain() { return hardware_pgain_;}
      double hardware_pgain() const { return hardware_pgain_;}
      //  ros::Time::now
      double dt() const { return dt_;}
      double& dt() { return dt_;}
      //  controller/state/desired
      double& command_angle() {return command_angle_;}
      double command_angle() const {return command_angle_;}

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

      double hardware_pgain_;
      double ulimit_;
      double llimit_;
      double uvlimit_;
      double lvlimit_;

      std::shared_ptr<JointLimitTable> jointLimitTable_;
      std::weak_ptr<JointInfo> jointLimitTableTargetJointInfo_;

      //cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> torqueSelectMatrix_;
    };

    void setupJointInfosFromParam(cnoid::Body* robot, std::vector<std::shared_ptr<JointInfo> >& jointInfos, std::map<std::string, std::shared_ptr<JointInfo> >& jointInfoMap);

    void setupJointInfoFromParam(const std::string& ns, std::shared_ptr<JointInfo>& jointinfo, std::map<std::string, std::shared_ptr<JointInfo> >& jointInfoMap);

  };
};

#endif
