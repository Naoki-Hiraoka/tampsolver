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

    cnoidbodyutils::setupJointInfosFromParam(robot_, jointInfos_, jointInfoMap_);

    PWTController_ = std::make_shared<PWTController>(robot_, jointInfos_);

    // setup PWTController from params
    {
      pnh.param("smooth_ratio",smooth_ratio_,2.0);
      pnh.param("sv_ratio",PWTController_->sv_ratio(),1e-12);
      pnh.param("k0",PWTController_->k0(),0.1);
      pnh.param("tolerance0_1",PWTController_->tolerance0_1(),0.02);
      pnh.param("k0_1",PWTController_->k0_1(),0.5);
      pnh.param("w0_1",PWTController_->w0_1(),1e-2);
      pnh.param("we0_1",PWTController_->we0_1(),1e6);
      pnh.param("k1",PWTController_->k1(),5.0);
      pnh.param("w1",PWTController_->w1(),1e-2);
      pnh.param("we1",PWTController_->we1(),1e4);
      pnh.param("w_scale1",PWTController_->w_scale1(),4e0);
      pnh.param("tau_scale1",PWTController_->tau_scale1(),4e0);
      pnh.param("tolerance1_1",PWTController_->tolerance1_1(),0.04);
      pnh.param("k1_1",PWTController_->k1_1(),0.5);
      pnh.param("w1_1",PWTController_->w1_1(),1e-2);
      pnh.param("we1_1",PWTController_->we1_1(),1e6);
      pnh.param("w2",PWTController_->w2(),1e-2);
      pnh.param("we2",PWTController_->we2(),1e4);
      pnh.param("w2_5",PWTController_->w2_5(),1e-2);
      pnh.param("we2_5",PWTController_->we2_5(),1e4);
      pnh.param("w_scale2_5",PWTController_->w_scale2_5(),4e0);
      pnh.param("k3",PWTController_->k3(),5.0);
      pnh.param("w3",PWTController_->w3(),1e1);
      pnh.param("w_scale3",PWTController_->w_scale3(),1e1);
      pnh.param("tau_scale3",PWTController_->tau_scale3(),1e1);
      pnh.param("w_weight3",PWTController_->w_weight3(),1e-1);
      pnh.param("tau_weight3",PWTController_->tau_weight3(),1e0);
      pnh.param("taumax_weight3",PWTController_->taumax_weight3(),1e1);
    }

    // setup subscribers
    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &PWTControllerROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber endEffectorsSub = nh.subscribe("end_effectors", 1, &PWTControllerROS::endEffectorsCallback, this);

    ros::Subscriber odomSub = nh.subscribe("odom", 1, &PWTControllerROS::odomCallback, this);

    ros::Subscriber motorStatesSub = nh.subscribe("motor_states", 1, &PWTControllerROS::motorStatesCallback, this);

    ros::Subscriber motorTemperatureStateSub = nh.subscribe("motor_temperature_states", 1, &PWTControllerROS::motorTemperatureStateCallback, this);

    ros::Subscriber controllerStateSub = nh.subscribe("fullbody_controller/state", 1, &PWTControllerROS::controllerStateCallback, this);

    ros::Subscriber selfCollisionSub = nh.subscribe("self_collision", 1, &PWTControllerROS::selfCollisionCallback, this);

    ros::Subscriber pclCollisionSub = nh.subscribe("pcl_collision", 1, &PWTControllerROS::pclCollisionCallback, this);

    ros::ServiceServer enableService = pnh.advertiseService("enable",&PWTControllerROS::enableCallback,this);

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction > controllerClient("fullbody_controller/follow_joint_trajectory_action", true);
    ROS_INFO("Waiting for fullbody_controller to start.");
    controllerClient.waitForServer();
    ROS_INFO("Action fullbody_controller started.");

    isEnabled_ = false;

    // main loop
    pnh.param("rate", this->rate_, 100.0); // 100 hz
    this->rosRate_ = std::make_shared<ros::Rate>(this->rate_);

    dynamic_reconfigure::Server<multicontact_controller_msgs::PWTControllerConfig> server;
    {
      multicontact_controller_msgs::PWTControllerConfig config = this->getCurrentConfig();
      server.setConfigDefault(config);
      server.updateConfig(config);
      server.setCallback(std::bind(&PWTControllerROS::configCallback, this, std::placeholders::_1, std::placeholders::_2));//この中でcallbackが呼ばれるので、その前にupdateConfigを呼ぶ必要がある
    }

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();
      double dt = std::min(10.0/this->rate_, std::max(1.0/this->rate_, (now - stamp).toSec())); //rosは厳密はdtは無理。dtが想定より小さすぎたり大きすぎると計算が不安定になるので。

      // spin
      ros::spinOnce();

      for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
        if(it->second->isValid()){
          it->second->contactPoint()->dt() = dt;
          it->second->contactPoint()->contact()->dt() = dt;
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
          for(size_t i=0;i<selfCollisions_.size();i++){
            std::vector<cnoid::SgNodePtr> objects = selfCollisions_[i]->getDrawOnObjects();
            for(size_t j=0;j<objects.size();j++){
              this->drawOn(objects[j]);
            }
          }
          for(size_t i=0;i<pclCollisions_.size();i++){
            std::vector<cnoid::SgNodePtr> objects = pclCollisions_[i]->getDrawOnObjects();
            for(size_t j=0;j<objects.size();j++){
              this->drawOn(objects[j]);
            }
          }
          this->flush();
        }

        // solve
        robot_->calcCenterOfMass();
        PWTController_->calcPWTControl(contactPoints, selfCollisions_, pclCollisions_, dt);

        // send command
        control_msgs::FollowJointTrajectoryGoal goal;
        goal.trajectory.header.seq = seq;
        goal.trajectory.header.stamp = now;
        goal.trajectory.points.resize(1);
        goal.trajectory.points[0].time_from_start = ros::Duration(dt * smooth_ratio_); // ちょっと長く
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
        std::shared_ptr<cnoidbodyutils::JointInfo> info = jointInfoMap_[msg->name[i]];
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

  void PWTControllerROS::selfCollisionCallback(const multicontact_controller_msgs::CollisionArray::ConstPtr& msg){
    cnoidbodyutils::collisionArrayMsgToCnoid(robot_,*msg, selfCollisions_);
  }

  void PWTControllerROS::pclCollisionCallback(const multicontact_controller_msgs::CollisionArray::ConstPtr& msg){
    cnoidbodyutils::collisionArrayMsgToCnoid(robot_,*msg, pclCollisions_);
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
    this->smooth_ratio_ = config.smooth_ratio;
    PWTController_->debug_print() = config.debug_print;
    PWTController_->sv_ratio() = config.sv_ratio;
    PWTController_->k0() = config.k0;
    PWTController_->tolerance0_1() = config.tolerance0_1;
    PWTController_->k0_1() = config.k0_1;
    PWTController_->w0_1() = config.w0_1;
    PWTController_->we0_1() = config.we0_1;
    PWTController_->k1() = config.k1;
    PWTController_->w1() = config.w1;
    PWTController_->we1() = config.we1;
    PWTController_->w_scale1() = config.w_scale1;
    PWTController_->tau_scale1() = config.tau_scale1;
    PWTController_->tolerance1_1() = config.tolerance1_1;
    PWTController_->k1_1() = config.k1_1;
    PWTController_->w1_1() = config.w1_1;
    PWTController_->we1_1() = config.we1_1;
    PWTController_->w2() = config.w2;
    PWTController_->we2() = config.we2;
    PWTController_->w2_5() = config.w2_5;
    PWTController_->we2_5() = config.we2_5;
    PWTController_->w_scale2_5() = config.w_scale2_5;
    PWTController_->k3() = config.k3;
    PWTController_->w3() = config.w3;
    PWTController_->w_scale3() = config.w_scale3;
    PWTController_->tau_scale3() = config.tau_scale3;
    PWTController_->w_weight3() = config.w_weight3;
    PWTController_->tau_weight3() = config.tau_weight3;
    PWTController_->taumax_weight3() = config.taumax_weight3;
  }

  multicontact_controller_msgs::PWTControllerConfig PWTControllerROS::getCurrentConfig(){
    multicontact_controller_msgs::PWTControllerConfig config;
    config.rate = this->rate_;
    config.smooth_ratio = this->smooth_ratio_;
    config.debug_print = PWTController_->debug_print();
    config.sv_ratio = PWTController_->sv_ratio();
    config.k0 = PWTController_->k0();
    config.tolerance0_1 = PWTController_->tolerance0_1();
    config.k0_1 = PWTController_->k0_1();
    config.w0_1 = PWTController_->w0_1();
    config.we0_1 = PWTController_->we0_1();
    config.k1 = PWTController_->k1();
    config.w1 = PWTController_->w1();
    config.we1 = PWTController_->we1();
    config.w_scale1 = PWTController_->w_scale1();
    config.tau_scale1 = PWTController_->tau_scale1();
    config.tolerance1_1 = PWTController_->tolerance1_1();
    config.k1_1 = PWTController_->k1_1();
    config.w1_1 = PWTController_->w1_1();
    config.we1_1 = PWTController_->we1_1();
    config.w2 = PWTController_->w2();
    config.we2 = PWTController_->we2();
    config.w2_5 = PWTController_->w2_5();
    config.we2_5 = PWTController_->we2_5();
    config.w_scale2_5 = PWTController_->w_scale2_5();
    config.k3 = PWTController_->k3();
    config.w3 = PWTController_->w3();
    config.w_scale3 = PWTController_->w_scale3();
    config.tau_scale3 = PWTController_->tau_scale3();
    config.w_weight3 = PWTController_->w_weight3();
    config.tau_weight3 = PWTController_->tau_weight3();
    config.taumax_weight3 = PWTController_->taumax_weight3();

    return config;
  }

};
