#include <multicontact_controller/PWTController/PWTControllerROS.h>

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>

namespace multicontact_controller {
  void EndEffectorPWTCROS::onInfoUpdated(){
    const std::string& linkname = info_->header.frame_id;
    cnoid::Link* link = cnoidbodyutils::getLinkFromURDFlinkName(robot_,linkname);
    if(!link) {
      ROS_WARN("Link '%s' is not found in %s",linkname.c_str(),robot_->name().c_str());
    }

    this->contactPoint()->parent() = link;
    cnoid::Vector3 p;
    tf::vectorMsgToEigen(info_->transform.translation,p);
    this->contactPoint()->T_local().translation() = p;
    Eigen::Quaterniond q;
    tf::quaternionMsgToEigen(info_->transform.rotation,q);
    this->contactPoint()->T_local().linear() = q.normalized().toRotationMatrix();
  }

  void EndEffectorPWTCROS::onStateUpdated(){
    if(prev_state_ != state_ &&
       (state_ == "CONTACT" || state_ == "TOWARD_BREAK_CONTACT") &&
       (prev_state_ != "CONTACT" && prev_state_ != "TOWARD_BREAK_CONTACT")){
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

    // setup subscribers
    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &PWTControllerROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber imuSub = nh.subscribe("imu", 1, &PWTControllerROS::imuCallback, this);

    ros::Subscriber endEffectorsSub = nh.subscribe("end_effectors", 1, &PWTControllerROS::endEffectorsCallback, this);

    ros::Publisher odomPub = nh.advertise<nav_msgs::Odometry>("multicontact_odom",10);
    tf2_ros::TransformBroadcaster br;

    ros::ServiceServer enableService = pnh.advertiseService("enable",&PWTControllerROS::enableCallback,this);
    ros::ServiceServer setBodyTransformService = pnh.advertiseService("set_body_transform",&PWTControllerROS::setBodyTransformCallback,this);

    pnh.param("start_enabled",isEnabled_,false);
    std::string base_frame;
    pnh.param("base_frame",base_frame,std::string("BODY"));
    std::string odom_frame;
    pnh.param("odom_frame",odom_frame,std::string("odom"));

    // main loop
    int rate;
    pnh.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
        std::vector<std::shared_ptr<ContactPointPWTC> > contactPoints;
        for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          if((it->second->state() == "CONTACT" || it->second->state() == "TOWARD_BREAK_CONTACT") &&
             it->second->contactPoint()->parent()){
            contactPoints.push_back(it->second->contactPoint());
          }
        }

        if(contactPoints.size() !=0){
          if(!multiContactFootCoords_.calcRootOdometry(robot_, contactPoints)){
            ROS_WARN("calcRootOdometry failed");
          }else{
            cnoid::Position originCoords = multiContactFootCoords_.getOriginCoords();
            for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
              it->second->originT() = originCoords;
            }
            Eigen::Affine3d odom = originCoords * robot_->rootLink()->T();
            nav_msgs::Odometry msg;
            msg.header.stamp = now;
            msg.header.frame_id = odom_frame;
            msg.header.seq = seq;
            msg.child_frame_id = base_frame;
            tf::poseEigenToMsg(odom, msg.pose.pose);
            odomPub.publish(msg);

            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = now;
            transformStamped.header.seq = seq;
            transformStamped.header.frame_id = odom_frame;
            transformStamped.child_frame_id = base_frame;
            tf::transformEigenToMsg(odom, transformStamped.transform);
            br.sendTransform(transformStamped);
          }
        }
      }

      seq++;
      stamp = now;

      drawObjects();

      r.sleep();
    }

    exit(0);

  }

  void PWTControllerROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(robot_){
      cnoidbodyutils::jointStateToBody(msg,robot_);
    }
  }

  void PWTControllerROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if(robot_){
      // rootのvel, accがない TODO
      cnoidbodyutils::imuToBody(msg,robot_);
    }
  }

  void PWTControllerROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
    endeffectorutils::stringArrayToEndEffectors(msg,endEffectors_,this->robot_);
  }

  bool PWTControllerROS::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    if(this->isEnabled_ == request.data){
      ROS_INFO("[IdleFootCoords::enableService] Already %s",(this->isEnabled_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    } else {
      this->isEnabled_ = request.data;
      ROS_INFO("[IdleFootCoords::enableService] %s",(this->isEnabled_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    }
  }

  bool PWTControllerROS::setBodyTransformCallback(multicontact_controller_msgs::SetTransformStamped::Request& request, multicontact_controller_msgs::SetTransformStamped::Response& response){
    Eigen::Affine3d bodyTransform;
    tf::transformMsgToEigen(request.transform.transform, bodyTransform);
    bool success = this->multiContactFootCoords_.setRootOdom(robot_, bodyTransform);

    if(success){
      cnoid::Position originCoords = this->multiContactFootCoords_.getOriginCoords();
      for(std::map<std::string,std::shared_ptr<EndEffectorPWTCROS> >::iterator it = endEffectors_.begin(); it != endEffectors_.end(); it++) {
        it->second->originT() = originCoords;
        if((it->second->state() == "CONTACT" || it->second->state() == "TOWARD_BREAK_CONTACT")){

        }
      }
    }

    response.success = success;
    response.message = "";

    return true;
  }
};
