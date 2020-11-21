#include <multicontact_controller/WalkController/WalkControllerROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/BodyLoader>
#include <std_srvs/SetBool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/TransformStamped.h>
#include <hrpsys_ros_bridge/OpenHRP_RobotHardwareService_getStatus2.h>
#include <hrpsys_ros_bridge/OpenHRP_StateHolderService_getCommand.h>
#include <multicontact_controller_msgs/SetTransformStamped.h>

namespace multicontact_controller {
  void WalkControllerROS::main(int argc, char** argv) {

    ros::init(argc,argv,"walk_controller");
    ros::NodeHandle n;
    ros::NodeHandle pn("~");

    // initialize robot
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

    // enable service
    pn.param("start_enabled", isActive_,true);
    ros::ServiceServer enableService  = pn.advertiseService("enable", &WalkControllerROS::enableCallback, this);

    // interface for AutoBalancer
    ros::Subscriber abcBaseTformSub =
    n.subscribe<nav_msgs::Odometry>
      ("odom",
       1,
       [&](const nav_msgs::Odometry::ConstPtr& msg){
        abcBaseTform_.header = msg->header;
        abcBaseTform_.child_frame_id = msg->child_frame_id;
        abcBaseTform_.transform.translation.x = msg->pose.pose.position.x;
        abcBaseTform_.transform.translation.y = msg->pose.pose.position.y;
        abcBaseTform_.transform.translation.z = msg->pose.pose.position.z;
        abcBaseTform_.transform.rotation = msg->pose.pose.orientation;
       });

    // wait for hrpsys_ros_bridge
    ros::ServiceClient rhGetStatus2Client = n.serviceClient<hrpsys_ros_bridge::OpenHRP_RobotHardwareService_getStatus2>("RobotHardwareServiceROSBridge/getStatus2");
    if(!rhGetStatus2Client.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("RobotHardwareServiceROSBridge/getStatus2").c_str());
      exit(1);
    }
    shGetCommandClient_ = n.serviceClient<hrpsys_ros_bridge::OpenHRP_StateHolderService_getCommand>("StateHolderServiceROSBridge/getCommand");
    if(!shGetCommandClient_.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("StateHolderServiceROSBridge/getCommand").c_str());
      exit(1);
    }

    // wait for simple_footcoords
    ros::ServiceClient simpleFootCoordsEnableClient = n.serviceClient<std_srvs::SetBool>("simple_footcoords/enable");
    if(!simpleFootCoordsEnableClient.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("simple_footcoords/enable").c_str());
      exit(1);
    }

    // wait for idle_footcoords
    ros::ServiceClient idleFootCoordsEnableClient = n.serviceClient<std_srvs::SetBool>("idle_footcoords/enable");
    if(!idleFootCoordsEnableClient.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("idle_footcoords/enable").c_str());
      exit(1);
    }
    ros::ServiceClient idleFootCoordsSetBodyTransformClient = n.serviceClient<multicontact_controller_msgs::SetTransformStamped>("idle_footcoords/set_body_transform");
    if(!idleFootCoordsSetBodyTransformClient.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("idle_footcoords/set_body_transform").c_str());
      exit(1);
    }
    ros::ServiceClient idleFootCoordsSetBaseFootPrintTransformClient = n.serviceClient<multicontact_controller_msgs::SetTransformStamped>("idle_footcoords/set_base_footprint_transform");
    if(!idleFootCoordsSetBaseFootPrintTransformClient.waitForExistence(ros::Duration(10))){
      ROS_ERROR("Service %s not found",n.resolveName("idle_footcoords/set_base_footprint_transform").c_str());
      exit(1);
    }


    // main loop
    int rate;
    pn.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();

    while (ros::ok()){

      // spin
      ros::spinOnce();

      ros::Time now = ros::Time::now();

      seq++;
      stamp = now;

      drawObjects();

      r.sleep();

    }

    exit(0);

  }

  bool WalkControllerROS::enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
    if(isActive_ == request.data){
      ROS_INFO("[WalkControllerROS::enableCallback] Already %s",(isActive_ ? "Enabled" : "Disabled"));
      response.success = true;
      response.message = "";
      return true;
    } else {
      if(request.data){
        if(onEnable()){
          isActive_ = true;
          ROS_INFO("[WalkControllerROS::enableCallback] Enabled");
          response.success = true;
          response.message = "";
          return true;
        } else {
          ROS_WARN("[WalkControllerROS::enableCallback] Enable failed");
          response.success = false;
          response.message = message_;
          return true;
        }
      } else {
        if(onDisable()){
          isActive_ = false;
          ROS_INFO("[WalkControllerROS::enableCallback] Disabled");
          response.success = true;
          response.message = "";
          return true;
        } else {
          ROS_WARN("[WalkControllerROS::enableCallback] Disable failed");
          response.success = false;
          response.message = message_;
          return true;
        }
      }
    }
  }

  bool WalkControllerROS::onEnable() {
    /*
      AutoBalancerがオフの間は、常にユーザーがrootlinkの位置姿勢をangle-vector-sequence-fullのpos+rpyで与えて、odom-BODYのtfを自分で管理する必要がある。
      AutoBalancerを入れる切り替えの際、odomは[今のStateHolderのbaseTform]から、[今のStateHolderの立ち位置で、重心が両足の中心になるようなangle-vectorのroot位置姿勢]へと線形補間される. 実機のangle-vectorは[今のStateHolderのangle-vector]から、[今のStateHolderの立ち位置で、重心が両足の中心になるようなangle-vector]へと線形補間される. 補間の間、angle-vectorと位置姿勢の非線形性から、odomは足の固定が考慮されない不正確な値になる。
      よって、重心が両足の中心の上にありAutoBalancerが修正する必要がないangle-vector+root位置姿勢をStateHolderに持たせてから、AutoBalancerを入れる必要がある。ここまではAutoBalancerがオフの間にユーザーがやってある前提
    */

    // get StateHolder values
    hrpsys_ros_bridge::OpenHRP_StateHolderService_getCommand shGetCommand;
    if(!shGetCommandClient_.call(shGetCommand)){
      message_ = "call" + shGetCommandClient_.getService() + " failed";
      ROS_WARN(message_.c_str());
      return false;
    }
    for(size_t i=0;i<shGetCommand.response.command.jointRefs.size();i++){
      if(robot_.joint(i)){
        robot_.joint(i)->q() = shGetCommand.response.command.jointRefs[i];
      }
    }
    robot_.rootLink()->p()(0) = shGetCommand.response.command.baseTransform[0];
    robot_.rootLink()->p()(1) = shGetCommand.response.command.baseTransform[1];
    robot_.rootLink()->p()(2) = shGetCommand.response.command.baseTransform[2];
    Eigen::AngleAxisd rollAngle(shGetCommand.response.command.baseTransform[0], Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(shGetCommand.response.command.baseTransform[1], Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(shGetCommand.response.command.baseTransform[2], Eigen::Vector3d::UnitZ());
    robot_.rootLink()->R() = yawAngle * pitchAngle * rollAngle;

    robot_->calcForwardKinematics(false,false);

    cnoid::Vector3 footMidPos;//TODO ここから
    cnoid::Vector3 centerOfMass = robot_->calcCenterOfMass();
    return true;
  }

  bool WalkControllerROS::onDisable() {
    return true;
  }
};
