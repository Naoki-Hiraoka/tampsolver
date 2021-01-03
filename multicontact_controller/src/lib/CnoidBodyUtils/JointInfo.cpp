#include <multicontact_controller/lib/CnoidBodyUtils/JointInfo.h>

#include <cnoid/EigenUtil>
#include <limits>
#include <ros/ros.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    JointInfo::JointInfo()
      : name_(),
        joint_(nullptr),
        controllable_(false),
        care_torque_(false),
        command_angle_(0.0),
        dt_(0.02),
        coil_temperature_limit_(150.0),
        housing_temperature_(25.0),
        coil_temperature_(25.0),
        maximum_effort_soft_(std::numeric_limits<double>::max()),
        maximum_effort_hard_(std::numeric_limits<double>::max()),
        balance_effort_(std::numeric_limits<double>::max()),
        remaining_time_(std::numeric_limits<double>::max()),
        pgain_(1e4),
        hardware_pgain_(1e0),
        ulimit_(std::numeric_limits<double>::max()),
        llimit_(-std::numeric_limits<double>::max()),
        uvlimit_(std::numeric_limits<double>::max()),
        lvlimit_(-std::numeric_limits<double>::max())
    {
    }

    // 指令関節角度上下限に関する制約を返す.破損防止
    void JointInfo::JointAngleConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc, bool use_command_angle){
      double ulimit = joint_->q_upper();
      double llimit = joint_->q_lower();

      double current_angle;
      if(use_command_angle) current_angle = command_angle_;
      else current_angle = joint_->q();

      // angle
      if(jointLimitTable_){
        double target_angle;
        if(use_command_angle) target_angle = jointLimitTableTargetJointInfo_.lock()->command_angle();
        else target_angle = jointLimitTableTargetJointInfo_.lock()->joint()->q();

        double min_angle = jointLimitTable_->getLlimit(target_angle) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
        double max_angle = jointLimitTable_->getUlimit(target_angle) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
        ulimit = std::max(llimit, std::min(ulimit, max_angle - current_angle)); //上限が下限を下回ることを防ぐ
        llimit = std::min(ulimit, std::max(llimit, min_angle - current_angle));
      }else{
        double min_angle = joint_->q_lower() + 0.0001;//少しマージン
        double max_angle = joint_->q_upper() - 0.0001;//少しマージン
        ulimit = std::max(llimit, std::min(ulimit, max_angle - current_angle)); //上限が下限を下回ることを防ぐ
        llimit = std::min(ulimit, std::max(llimit, min_angle - current_angle));
      }

      A.resize(0,1);
      b.resize(0);
      wa.resize(0);
      C.resize(1,1);C.coeffRef(0,0) = 1.0;
      dl.resize(1);dl[0] = llimit;
      du.resize(1);du[0] = ulimit;
      wc.resize(1);wc[0] = 1.0;
      return;
    }

    // 指令関節角速度上下限に関する制約を返す.破損防止
    void JointInfo::JointVelocityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
      // velocity
      double ulimit = joint_->q_upper() * dt_;
      double llimit = joint_->q_lower() * dt_;
      ulimit = std::max(llimit, std::min(ulimit, joint_->dq_upper() * dt_ - 0.000175));// 0.01 deg / sec (same as SoftErrorLimiter)
      llimit = std::min(ulimit, std::max(llimit, joint_->dq_lower() * dt_ + 0.000175));// 0.01 deg / sec (same as SoftErrorLimiter)
      ulimit = std::max(llimit, std::min(ulimit, uvlimit_)); //上限が下限を下回ることを防ぐ
      llimit = std::min(ulimit, std::max(llimit, lvlimit_));

      A.resize(0,1);
      b.resize(0);
      wa.resize(0);
      C.resize(1,1);C.coeffRef(0,0) = 1.0;
      dl.resize(1);dl[0] = llimit;
      du.resize(1);du[0] = ulimit;
      wc.resize(1);wc[0] = 1.0;
      return;
    }

    // M \tau によってこのJointの成分を抽出できるM(select matrix). \tauは[numJoints]
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& JointInfo::torqueSelectMatrix(){
      if(torqueSelectMatrix_.cols() != joint_->body()->numJoints() || torqueSelectMatrix_.rows() != 1){
        torqueSelectMatrix_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,joint_->body()->numJoints());
        torqueSelectMatrix_.insert(0,joint_->jointId()) = 1.0;
      }
      return torqueSelectMatrix_;
    }

    // 関節トルク上下限に関する制約を返す.破損防止
    // 各行は無次元化されている
    void JointInfo::JointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
      if(care_torque_){
        double max_torque = joint_->info<double>("climit") * joint_->info<double>("torqueConst") * joint_->info<double>("gearRatio");
        double scale = std::max(std::min(maximum_effort_hard_, max_torque), 1e-4);
        A.resize(0,1);
        b.resize(0);
        wa.resize(0);
        C.resize(1,1); C.coeffRef(0,0) = 1.0/scale;
        dl.resize(1);
        du.resize(1);
        wc.resize(1);

        du[0] = std::min(maximum_effort_hard_, max_torque)/scale - C.coeffRef(0,0) * joint_->u();
        dl[0] = std::max(-maximum_effort_hard_, -max_torque)/scale - C.coeffRef(0,0) * joint_->u();
        wc[0] = 1.0;
      }else{
        A.resize(0,1);
        b.resize(0);
        wa.resize(0);
        C.resize(0,1);
        dl.resize(0);
        du.resize(0);
        wc.resize(0);
      }
      return;
    }

    // 各行は無次元化されている
    void JointInfo::bestEffortJointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      if(care_torque_){
        double max_torque = joint_->info<double>("climit") * joint_->info<double>("torqueConst") * joint_->info<double>("gearRatio");
        double scale = std::max(std::min(maximum_effort_soft_, max_torque), 1e-4);
        A.resize(1,1); A.coeffRef(0,0) = 1.0/scale;
        b.resize(1);
        wa.resize(1);
        C.resize(0,1);
        dl.resize(0);
        du.resize(0);
        wc.resize(0);

        b[0] = 0 - A.coeffRef(0,0) * joint_->u();
        wa[0] = 1.0;
      }else{
        A.resize(0,1);
        b.resize(0);
        wa.resize(0);
        C.resize(0,1);
        dl.resize(0);
        du.resize(0);
        wc.resize(0);
      }

      return;
    }

    void setupJointInfosFromParam(cnoid::Body* robot, std::vector<std::shared_ptr<JointInfo> >& jointInfos, std::map<std::string, std::shared_ptr<JointInfo> >& jointInfoMap){
      for(size_t i=0;i<robot->numJoints();i++){
        std::string name = robot->joint(i)->name();
        std::shared_ptr<JointInfo> jointInfo = std::make_shared<JointInfo>();
        jointInfo->name() = name;
        jointInfo->joint() = robot->joint(i);
        jointInfos.push_back(jointInfo);
        jointInfoMap[name] = jointInfo;
      }
      for(size_t i=0;i<jointInfos.size();i++){
        setupJointInfoFromParam("joint_config/"+jointInfos[i]->name(), jointInfos[i], jointInfoMap);
      }

    }

    void setupJointInfoFromParam(const std::string& ns, std::shared_ptr<JointInfo>& jointInfo, std::map<std::string, std::shared_ptr<JointInfo> >& jointInfoMap){
      ros::NodeHandle nh;

      if(!nh.hasParam(ns+"/controllable")){
        ROS_WARN("rosparam %s not found",(ns+"/controllable").c_str());
      }else{
        nh.getParam(ns+"/controllable",jointInfo->controllable());
      }
      if(!nh.hasParam(ns+"/care_torque")){
        ROS_WARN("rosparam %s not found",(ns+"/care_torque").c_str());
      }else{
        nh.getParam(ns+"/care_torque",jointInfo->care_torque());
      }
      if(!nh.hasParam(ns+"/hardware_pgain")){
        ROS_WARN("rosparam %s not found",(ns+"/hardware_pgain").c_str());
      }else{
        nh.getParam(ns+"/hardware_pgain",jointInfo->hardware_pgain());
        jointInfo->pgain() = jointInfo->hardware_pgain();
      }


      std::string limits_ns = ns + "/limits";

      if(nh.hasParam(limits_ns+"/ulimit")){
        ROS_INFO("rosparam %s found",(ns+"/ulimit").c_str());
        nh.getParam(limits_ns+"/ulimit",jointInfo->ulimit());
      }
      if(nh.hasParam(limits_ns+"/llimit")){
        ROS_INFO("rosparam %s found",(ns+"/llimit").c_str());
        nh.getParam(limits_ns+"/llimit",jointInfo->llimit());
      }
      if(nh.hasParam(limits_ns+"/uvlimit")){
        ROS_INFO("rosparam %s found",(ns+"/uvlimit").c_str());
        nh.getParam(limits_ns+"/uvlimit",jointInfo->uvlimit());
      }
      if(nh.hasParam(limits_ns+"/lvlimit")){
        ROS_INFO("rosparam %s found",(ns+"/lvlimit").c_str());
        nh.getParam(limits_ns+"/lvlimit",jointInfo->lvlimit());
      }

      if(nh.hasParam(limits_ns+"/limit_table/target_joint") &&
         nh.hasParam(limits_ns+"/limit_table/target_llimit_angle") &&
         nh.hasParam(limits_ns+"/limit_table/target_ulimit_angle") &&
         nh.hasParam(limits_ns+"/limit_table/llimit_table") &&
         nh.hasParam(limits_ns+"/limit_table/ulimit_table")){
        ROS_INFO("load limit table %s",(limits_ns+"/limit_table").c_str());
        std::string target_joint_name;
        nh.getParam(limits_ns+"/limit_table/target_joint",target_joint_name);
        int target_llimit_angle;
        nh.getParam(limits_ns+"/limit_table/target_llimit_angle",target_llimit_angle);
        int target_ulimit_angle;
        nh.getParam(limits_ns+"/limit_table/target_ulimit_angle",target_ulimit_angle);
        std::vector<double> llimit_table;
        nh.getParam(limits_ns+"/limit_table/llimit_table",llimit_table);
        std::vector<double> ulimit_table;
        nh.getParam(limits_ns+"/limit_table/ulimit_table",ulimit_table);
        cnoid::Link* self_joint = jointInfo->joint();
        cnoid::Link* target_joint = jointInfo->joint()->body()->link(target_joint_name);
        std::shared_ptr<JointInfo> target_joint_info = jointInfoMap[target_joint_name];
        if(!target_joint || !target_joint_info){
          ROS_ERROR("target_joint %s not found", target_joint_name.c_str());
        }else{
          jointInfo->jointLimitTable() = std::make_shared<cnoidbodyutils::JointLimitTable>(self_joint,
                                                                                           target_joint,
                                                                                           target_llimit_angle,
                                                                                           target_ulimit_angle,
                                                                                           llimit_table,
                                                                                           ulimit_table);
          jointInfo->jointLimitTableTargetJointInfo() = target_joint_info;
        }
      }

    }

  }
}
