#include <multicontact_controller/PWTController/PWTControllerROS.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

namespace multicontact_controller {
  void EndEffectorPWTCROS::onInfoUpdated(){
    endeffectorutils::updateContactPointFromInfo(robot_, contactPoint_, *info_);
    cnoidbodyutils::loadContactFromInfo(info_->contact, contactPoint_->contact());
    cnoidbodyutils::loadInteractionFromInfo(info_->interaction, contactPoint_->interaction());
  }

  void EndEffectorPWTCROS::onStateUpdated(){
    if(this->isValid()){
      contactPoint_->state() = state_;
      if(prev_state_ != state_ &&
         (state_ == "AIR" || state_ == "NEAR_CONTACT" || state_ == "TOWARD_MAKE_CONTACT") &&
         (prev_state_ != "AIR" && prev_state_ != "NEAR_CONTACT" && prev_state_ != "TOWARD_MAKE_CONTACT")){
        contactPoint_->interaction()->reset_ref();
      }
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

  void PWTControllerROS::main(int argc, char** argv) {

    ros::init(argc,argv,"pwt_controller");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    robot_ = cnoidbodyutils::loadBodyFromParam("/vrml_file");
    if(!robot_){
      ROS_ERROR("Failed to load robot from '/vrml_file'");
      return;
    }
    objects(robot_);

    for(size_t i=0;i<robot_->numJoints();i++){
      std::string name = robot_->joint(i)->name();
      std::shared_ptr<JointInfo> jointInfo = std::make_shared<JointInfo>();
      jointInfo->name() = name;
      jointInfo->joint() = robot_->joint(i);
      jointInfos_.push_back(jointInfo);
      jointInfoMap_[name] = jointInfo;
    }
    for(size_t i=0;i<jointInfos_.size();i++){
      setupJointInfoFromParam("joint_config/"+jointInfos_[i]->name(), jointInfos_[i], jointInfoMap_);
    }

    PWTController_ = std::make_shared<PWTController>(robot_, jointInfos_);

    // setup subscribers
    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &PWTControllerROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber endEffectorsSub = nh.subscribe("end_effectors", 1, &PWTControllerROS::endEffectorsCallback, this);

    ros::Subscriber odomSub = nh.subscribe("multicontact_odom", 1, &PWTControllerROS::odomCallback, this);

    ros::Subscriber motorStatesSub = nh.subscribe("motor_states", 1, &PWTControllerROS::motorStatesCallback, this);

    ros::Subscriber motorTemperatureStateSub = nh.subscribe("motor_temperature_states", 1, &PWTControllerROS::motorTemperatureStateCallback, this);

    ros::Subscriber controllerStateSub = nh.subscribe("fullbody_controller/state", 1, &PWTControllerROS::controllerStateCallback, this);

    ros::ServiceServer enableService = pnh.advertiseService("enable",&PWTControllerROS::enableCallback,this);

    dynamic_reconfigure::Server<multicontact_controller_msgs::PWTControllerConfig> server;
    server.setCallback(std::bind(&PWTControllerROS::configCallback, this, std::placeholders::_1, std::placeholders::_2));
    this->updateServerConfig(server);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction > controllerClient("fullbody_controller/follow_joint_trajectory_action", true);
    ROS_INFO("Waiting for fullbody_controller to start.");
    controllerClient.waitForServer();
    ROS_INFO("Action fullbody_controller started.");

    isEnabled_ = false;

    // main loop
    pnh.param("rate", this->rate_, 50.0); // 50 hz
    this->rosRate_ = std::make_shared<ros::Rate>(this->rate_);

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();
      double dt = std::min(10.0/this->rate_, std::max(1.0/this->rate_, (now - stamp).toSec())); //rosは厳密はdtは無理。dtが想定より小さすぎたり大きすぎると計算が不安定になるので。

      // spin
      ros::spinOnce();

      for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
        if(it->second->isValid()){
          it->second->contactPoint()->interaction()->T() = it->second->contactPoint()->parent()->T() * it->second->contactPoint()->T_local();
          it->second->contactPoint()->interaction()->dt() = dt;
        }
      }

      for(size_t i=0;i<jointInfos_.size();i++){
        jointInfos_[i]->dt() = dt;
      }

      if( !this->isEnabled_ ){
        this->drawObjects();
      } else {
        std::vector<std::shared_ptr<ContactPointPWTC> > contactPoints;
        for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          if(it->second->state() != "NOT_CARED" &&
             it->second->isValid()){
            contactPoints.push_back(it->second->contactPoint());
          }
        }

        // draw
        if(this->hasViewer()){
          this->drawObjects(false);
          for(size_t i=0;i<contactPoints.size();i++){
            std::vector<cnoid::SgNodePtr> objects = contactPoints[i]->getDrawOnObjects();
            for(size_t j=0;j<objects.size();j++){
              this->drawOn(objects[j]);
            }
          }
          this->flush();
        }

        // solve
        robot_->calcCenterOfMass();
        PWTController_->calcPWTControl(contactPoints, dt);

        // send command
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.seq = seq;
        goal.trajectory.header.stamp = now;
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].time_from_start = ros::Duration(dt * 1.1); // ちょっと長く
        for(size_t i=0;i<jointInfos_.size();i++){
          if(jointInfos_[i]->controllable()){
            goal.trajectory.joint_names.push_back(jointInfos_[i]->name());
            goal.trajectory.points[0].positions.push_back(jointInfos_[i]->command_angle());
          }
        }
        controllerClient.sendGoal(goal);
      }

      seq++;
      stamp = now;

      this->rosRate_->sleep();
    }

    exit(0);

  }

  void PWTControllerROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(robot_){
      cnoidbodyutils::jointStateToBody(msg,robot_);
    }
  }

  void PWTControllerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if(robot_){
      cnoidbodyutils::odomToBody(msg, robot_);
    }
  }

  void PWTControllerROS::motorStatesCallback(const hrpsys_ros_bridge::MotorStates::ConstPtr& msg){
    if(msg->name.size() == msg->pgain.size()){
      for(size_t i=0;i<msg->name.size();i++){
        if(jointInfoMap_.find(msg->name[i]) != jointInfoMap_.end()){
          jointInfoMap_[msg->name[i]]->pgain() = jointInfoMap_[msg->name[i]]->hardware_pgain() * msg->pgain[i];
        }
      }
    }
  }

  void PWTControllerROS::motorTemperatureStateCallback(const multicontact_controller_msgs::MotorTemperatureState::ConstPtr& msg){
    for(size_t i=0;i<msg->name.size();i++){
      if(jointInfoMap_.find(msg->name[i]) != jointInfoMap_.end()){
        std::shared_ptr<JointInfo> info = jointInfoMap_[msg->name[i]];
        if(msg->coil_temperature_limit.size() == msg->name.size()) info->coil_temperature_limit() = msg->coil_temperature_limit[i];
        if(msg->housing_temperature.size() == msg->name.size()) info->housing_temperature() = msg->housing_temperature[i];
        if(msg->coil_temperature.size() == msg->name.size()) info->coil_temperature() = msg->coil_temperature[i];
        if(msg->maximum_effort_soft.size() == msg->name.size()) info->maximum_effort_soft() = msg->maximum_effort_soft[i];
        if(msg->maximum_effort_hard.size() == msg->name.size()) info->maximum_effort_hard() = msg->maximum_effort_hard[i];
        if(msg->balance_effort.size() == msg->name.size()) info->balance_effort() = msg->balance_effort[i];
        if(msg->remaining_time.size() == msg->name.size()) info->remaining_time() = msg->remaining_time[i];
      }
    }
  }

  void PWTControllerROS::controllerStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg){
    controllerState_ = msg;
  }

  void PWTControllerROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
    endeffectorutils::stringArrayToEndEffectors(msg,endEffectors_,this->robot_);
  }

  bool PWTControllerROS::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    if(this->isEnabled_ == request.data){
      ROS_INFO("[PWTController::enableService] Already %s",(this->isEnabled_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    } else {
      if(request.data && controllerState_){
        // 指令関節角度を初期化
        for(size_t i=0;i<controllerState_->joint_names.size();i++){
          std::string name = controllerState_->joint_names[i];
          double command_angle = controllerState_->desired.positions[i];
          if(jointInfoMap_.find(name) != jointInfoMap_.end()){
            jointInfoMap_[name]->command_angle() = command_angle;
          }
        }

        // 目標エンドエフェクタ位置を初期化
        for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          if(it->second->isValid()){
            it->second->contactPoint()->interaction()->reset_ref();
          }
        }
      }
      this->isEnabled_ = request.data;
      ROS_INFO("[PWTController::enableService] %s",(this->isEnabled_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    }
  }

  void PWTControllerROS::configCallback(multicontact_controller_msgs::PWTControllerConfig& config, int32_t level){
    if(this->rate_ != config.rate){
      this->rate_ = config.rate;
      this->rosRate_ = std::make_shared<ros::Rate>(this->rate_);
    }
    PWTController_->sv_ratio() = config.sv_ratio;
    PWTController_->k0() = config.k0;
    PWTController_->k1() = config.k1;
    PWTController_->w1() = config.w1;
    PWTController_->we1() = config.we1;
    PWTController_->w_scale1() = config.w_scale1;
    PWTController_->tau_scale1() = config.tau_scale1;
    PWTController_->w2() = config.w2;
    PWTController_->we2() = config.we2;
    PWTController_->k3() = config.k3;
    PWTController_->w3() = config.w3;
    PWTController_->w_scale3() = config.w_scale3;
    PWTController_->tau_scale3() = config.tau_scale3;
    PWTController_->taumax_weight3() = config.taumax_weight3;
  }

  void PWTControllerROS::updateServerConfig(dynamic_reconfigure::Server<multicontact_controller_msgs::PWTControllerConfig>& server){
    multicontact_controller_msgs::PWTControllerConfig config;
    config.rate = this->rate_;
    config.sv_ratio = PWTController_->sv_ratio();
    config.k0 = PWTController_->k0();
    config.k1 = PWTController_->k1();
    config.w1 = PWTController_->w1();
    config.we1 = PWTController_->we1();
    config.w_scale1 = PWTController_->w_scale1();
    config.tau_scale1 = PWTController_->tau_scale1();
    config.w2 = PWTController_->w2();
    config.we2 = PWTController_->we2();
    config.k3 = PWTController_->k3();
    config.w3 = PWTController_->w3();
    config.w_scale3 = PWTController_->w_scale3();
    config.tau_scale3 = PWTController_->tau_scale3();
    config.taumax_weight3 = PWTController_->taumax_weight3();
    server.updateConfig(config);
  }

};
