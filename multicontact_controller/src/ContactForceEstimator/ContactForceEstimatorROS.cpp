#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/ForceSensor>
#include <cnoid/BodyLoader>

namespace multicontact_controller {
  void EndEffectorCFEROS::infoCallback(const multicontact_controller_msgs::EndEffectorInfo::ConstPtr& msg){
    //choreonoidのロボットモデルはリンク名が関節名によって管理されている
    const std::string& linkname = msg->header.frame_id;
    if(jointLinkMap_.find(linkname)==jointLinkMap_.end()){
      jointLinkMap_[linkname] = nullptr;
      for(size_t i=0;i<robot_->links().size();i++){
        cnoid::Affine3 tmp;
        cnoid::SgNodePath path = robot_->links()[i]->visualShape()->findNode(linkname,tmp);
        if(path.size()!=0){
          jointLinkMap_[linkname] = robot_->links()[i];
          break;
        }
      }
    }
    cnoid::Link* link = jointLinkMap_[linkname];
    if(!link) {
      ROS_WARN("Link '%s' not found",linkname.c_str());
    }

    this->linkName() = linkname;
    this->contactPoint()->parent() = link;
    cnoid::Vector3 p;
    tf::vectorMsgToEigen(msg->transform.translation,p);
    this->contactPoint()->T_local().translation() = p;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(msg->transform.rotation,q);
    this->contactPoint()->T_local().linear() = q.normalized().toRotationMatrix();
  }

  void ContactForceEstimatorROS::main(int argc, char** argv) {

    ros::init(argc,argv,"contact_force_estimator");
    ros::NodeHandle n;
    ros::NodeHandle nl("~");

    // initialize robot
    // 質量パラメータのキャリブ TODO
    std::string vrml_file;
    if (!n.getParam("/vrml_file", vrml_file)) {
      ROS_ERROR("Failed to get param 'vrml_file'");
      return;
    }
    // package://に対応
    std::string packagestr = "package://";
    if(vrml_file.size()>packagestr.size() && vrml_file.substr(0,packagestr.size()) == packagestr){
      vrml_file = vrml_file.substr(packagestr.size());
      int pos = vrml_file.find("/");
      vrml_file = ros::package::getPath(vrml_file.substr(0,pos)) + vrml_file.substr(pos);
    }
    cnoid::BodyLoader loader;
    robot_ = loader.load(vrml_file);
    if(!robot_){
      ROS_ERROR("Failed to load %s", vrml_file.c_str());
      return;
    }
    objects(robot_);

    // setup subscribers
    ros::Subscriber jointStateSub = n.subscribe("joint_states", 100, &ContactForceEstimatorROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber imuSub = n.subscribe("imu", 1, &ContactForceEstimatorROS::imuCallback, this);

    std::vector<ros::Subscriber> forceSensorSub;
    {
      cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
      for(size_t i=0;i<forceSensors.size();i++){
        forceSensorSub.push_back(n.subscribe(forceSensors[i]->name(), 1, &ContactForceEstimatorROS::forceSensorCallback, this));
      }
    }

    ros::Subscriber endEffectorsSub = n.subscribe("end_effectors", 1, &ContactForceEstimatorROS::endEffectorsCallback, this);

    // setup publishers
    std::map<std::string,ros::Publisher> offsetForcePub;
    if(robot_){
      cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
      for(size_t i=0;i<forceSensors.size();i++){
        offsetForcePub[forceSensors[i]->name()] = n.advertise<geometry_msgs::WrenchStamped>(forceSensors[i]->name()+"_offset", 1000);
        std::vector<double> initial_offset;
        n.param(forceSensors[i]->name()+"/initial_offset",initial_offset,std::vector<double>());
        cnoid::Vector6 offset;
        if(initial_offset.size() == 6){
          for(size_t i=0;i<6;i++)offset[i] = initial_offset[i];
        }else{
          offset = cnoid::Vector6::Zero();
        }
        forceSensorOffsets_[forceSensors[i]->name()] = offset;
      }
    }

    // setup ContactForceEstimator
    ContactForceEstimator contactForceEstimator;
    contactForceEstimator.setRobot(robot_);

    ros::ServiceServer enableService = nl.advertiseService("enable",&ContactForceEstimatorROS::enableCallback,this);

    double offsetUpdaterate;
    nl.param("offset_update_rate", offsetUpdaterate, 1.0); // 1 hz
    double forceOffsetUpdateThre;
    nl.param("force_offset_update_thre", forceOffsetUpdateThre, 0.5); //0.5[N]
    double momentOffsetUpdateThre;
    nl.param("moment_offset_update_thre", momentOffsetUpdateThre, 0.1); //0.1 [Nm]

    // main loop
    int rate;
    nl.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
        // update candidatepoints
        contactForceEstimator.clearCandidatePoints();
        for(std::map<std::string,std::shared_ptr<EndEffectorCFEROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          if(it->second->contactPoint()->parent() &&
             (it->second->state() == "NEAR_CONTACT" ||
              it->second->state() == "TOWARD_BREAK_CONTACT" ||
              it->second->state() == "CONTACT" ||
              it->second->state() == "TOWARD_MAKE_CONTACT")){
            contactForceEstimator.setCandidatePoint(it->second->contactPoint());
          }
        }

        //estimate
        contactForceEstimator.estimateForce();

        //publish
        for(std::map<std::string,std::shared_ptr<EndEffectorCFEROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          cnoid::Vector6 F = contactForceEstimator.getEstimatedForce(it->first);

          geometry_msgs::WrenchStamped msg;
          msg.header.seq = seq;
          msg.header.stamp = now;
          msg.header.frame_id = it->first;
          tf::wrenchEigenToMsg(F, msg.wrench);
          it->second->contactForcePub().publish(msg);
        }

        cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
        for(size_t i=0;i<forceSensors.size();i++){
          cnoid::Vector6 F = contactForceEstimator.getOffsetForce(forceSensors[i]->name());
          if(F.head<3>().norm() > forceOffsetUpdateThre) {
            forceSensorOffsets_[forceSensors[i]->name()].head<3>() += 1.0/offsetUpdaterate*(now-stamp).toSec() * F.head<3>();
          }
          if(F.tail<3>().norm() > momentOffsetUpdateThre) {
            forceSensorOffsets_[forceSensors[i]->name()].tail<3>() += 1.0/offsetUpdaterate*(now-stamp).toSec() * F.tail<3>();
          }
          geometry_msgs::WrenchStamped msg;
          msg.header.seq = seq;
          msg.header.stamp = now;
          msg.header.frame_id = forceSensors[i]->name();
          tf::wrenchEigenToMsg(forceSensorOffsets_[forceSensors[i]->name()], msg.wrench);
          offsetForcePub[forceSensors[i]->name()].publish(msg);
        }

      }

      seq++;
      stamp = now;

      drawObjects();

      r.sleep();
    }

    exit(0);

  }

  void ContactForceEstimatorROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 加速度がない TODO
    if(robot_){
      for(size_t i=0;i<msg->name.size();i++){
        cnoid::Link* joint = robot_->link(msg->name[i]);
        if(!joint) continue;
        if(msg->position.size() == msg->name.size()) joint->q() = msg->position[i];
        if(msg->velocity.size() == msg->name.size()) joint->dq() = msg->velocity[i];
        if(msg->effort.size() == msg->name.size()) joint->u() = msg->effort[i];
      }

      robot_->calcForwardKinematics(false,false);
    }
  }

  void ContactForceEstimatorROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if(robot_){
      // rootのvel, accがない TODO
      cnoid::Device* device = robot_->findDevice(msg->header.frame_id);
      if(!device) return;
      cnoid::Position T = device->link()->T() * device->T_local();
      cnoid::Position realT; realT.setIdentity();
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(msg->orientation,q);
      realT.linear() = q.normalized().toRotationMatrix();
      cnoid::Position nextT = realT * T.inverse() * robot_->rootLink()->T();
      robot_->rootLink()->T() = nextT;
      robot_->calcForwardKinematics(false,false);
    }
  }

  void ContactForceEstimatorROS::forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    if(robot_){
      cnoid::ForceSensor* device = robot_->findDevice<cnoid::ForceSensor>(msg->header.frame_id);
      if(!device) return;
      cnoid::Vector6 F;
      tf::wrenchMsgToEigen(msg->wrench,F);
      device->F() = F - forceSensorOffsets_[msg->header.frame_id];
    }
  }

  void ContactForceEstimatorROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string,std::shared_ptr<EndEffectorCFEROS> >::iterator it = endEffectors_.begin(); it != endEffectors_.end(); ) {
      if (std::find_if(msg->strings.begin(),msg->strings.end(),[&](std::string x){return x==it->first;}) == msg->strings.end()) {
        it = endEffectors_.erase(it);
      }
      else {
        ++it;
      }
    }

    // 増加したEndEffectorの反映
    for(size_t i=0;i<msg->strings.size();i++){
      if(endEffectors_.find(msg->strings[i])==endEffectors_.end()){
        endEffectors_[msg->strings[i]] = std::make_shared<EndEffectorCFEROS>(msg->strings[i], this->robot_);
      }
    }
  }

  bool ContactForceEstimatorROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
