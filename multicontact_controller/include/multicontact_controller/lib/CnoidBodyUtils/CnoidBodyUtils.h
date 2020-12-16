#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_H

#include <cnoid/Body>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    // rosparamのファイル名からロードする
    cnoid::Body* loadBodyFromParam(const std::string& paramname);

    //choreonoidのロボットモデルはリンク名が関節名によって管理されている
    cnoid::Link* getLinkFromURDFlinkName(cnoid::Body* robot, const std::string& linkname);

    void jointStateToBody(const sensor_msgs::JointState::ConstPtr& msg, cnoid::Body* robot);

    void imuToBody(const sensor_msgs::Imu::ConstPtr& msg, cnoid::Body* robot);
  };
};

#endif
