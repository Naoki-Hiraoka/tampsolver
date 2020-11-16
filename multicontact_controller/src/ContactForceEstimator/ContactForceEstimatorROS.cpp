#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/ForceSensor>
#include <cnoid/BodyLoader>

namespace multicontact_controller {
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
    if(robot_){
      cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
      for(size_t i=0;i<forceSensors.size();i++){
        forceSensorSub.push_back(n.subscribe(forceSensors[i]->name(), 1, &ContactForceEstimatorROS::forceSensorCallback, this));
      }
    }

    ros::Subscriber contactPointsSub = n.subscribe("end_effector_states", 1, &ContactForceEstimatorROS::contactPointsCallback, this);

    // setup publishers
    std::map<std::string,ros::Publisher> contactForcePub;


    // setup ContactForceEstimator
    ContactForceEstimator contactForceEstimator;
    contactForceEstimator.setRobot(robot_);


    // main loop
    int rate;
    nl.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    unsigned int seq = 0;
    while (ros::ok()) {

      // spin
      ros::spinOnce();

      // update candidatepoints
      contactForceEstimator.clearCandidatePoints();
      for(std::map<std::string,std::shared_ptr<EndEffectorState> >::iterator it=endEffectorStates_.begin();it!=endEffectorStates_.end();it++){
        switch(it->second->state()){
        case multicontact_controller_msgs::EndEffectorState::NOT_CARED :
        case multicontact_controller_msgs::EndEffectorState::AIR :
          break;
        default:
          contactForceEstimator.setCandidatePoint(it->second->contactPoint());
          break;
        }
      }

      //estimate
      contactForceEstimator.estimateForce();

      //publish
      ros::Time now = ros::Time::now();
      for(std::map<std::string,std::shared_ptr<EndEffectorState> >::iterator it=endEffectorStates_.begin();it!=endEffectorStates_.end();it++){
        cnoid::Vector6 F = contactForceEstimator.getEstimatedForce(it->first);

        geometry_msgs::WrenchStamped msg;
        msg.header.seq = seq;
        msg.header.stamp = now;
        msg.header.frame_id = it->first;
        tf::wrenchEigenToMsg(F, msg.wrench);
        it->second->contactForcePub().publish(msg);
      }
      seq++;

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
      cnoid::Position realT;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(msg->orientation,q);
      realT.linear() = q.normalized().toRotationMatrix();
      robot_->rootLink()->T() = realT * T.inverse() * robot_->rootLink()->T();
      robot_->calcForwardKinematics(false,false);
    }
  }

  void ContactForceEstimatorROS::forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    if(robot_){
      cnoid::ForceSensor* device = robot_->findDevice<cnoid::ForceSensor>(msg->header.frame_id);
      if(!device) return;
      tf::wrenchMsgToEigen(msg->wrench,device->F());
    }
  }

  void ContactForceEstimatorROS::contactPointsCallback(const multicontact_controller_msgs::EndEffectorStateArray::ConstPtr& msg) {
    // 消滅したEndEffectorを削除
    for(std::map<std::string,std::shared_ptr<EndEffectorState> >::iterator it = endEffectorStates_.begin(); it != endEffectorStates_.end(); ) {
      if (std::find_if(msg->endeffectorstates.begin(),msg->endeffectorstates.end(),[&](multicontact_controller_msgs::EndEffectorState x){return x.name==it->first;}) == msg->endeffectorstates.end()) {
        it = endEffectorStates_.erase(it);
      }
      else {
        ++it;
      }
    }

    // EndEffectorの反映
    for(size_t i=0;i<msg->endeffectorstates.size();i++){
      //choreonoidのロボットモデルはリンク名が関節名によって管理されている
      const std::string& linkname = msg->endeffectorstates[i].header.frame_id;
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
        continue;
      }


      // 反映
      if(endEffectorStates_.find(msg->endeffectorstates[i].name)==endEffectorStates_.end()){
        endEffectorStates_[msg->endeffectorstates[i].name] = std::make_shared<EndEffectorState>(msg->endeffectorstates[i].name);
      }
      std::shared_ptr<EndEffectorState> eef = endEffectorStates_[msg->endeffectorstates[i].name];

      eef->linkName() = linkname;
      eef->state() = msg->endeffectorstates[i].state;

      eef->contactPoint()->name() = msg->endeffectorstates[i].name;
      eef->contactPoint()->parent() = link;
      cnoid::Vector3 p;
      tf::vectorMsgToEigen(msg->endeffectorstates[i].transform.translation,p);
      eef->contactPoint()->T_local().translation() = p;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(msg->endeffectorstates[i].transform.rotation,q);
      eef->contactPoint()->T_local().linear() = q.normalized().toRotationMatrix();
    }
  }
};
