#ifndef CONTACT_FORCE_ESTIMATOR_ROS_H
#define CONTACT_FORCE_ESTIMATOR_ROS_H

#include <multicontact_controller/ContactForceEstimator/ContactForceEstimator.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <ros/ros.h>

namespace multicontact_controller {
  class EndEffectorCFEROS {
  public:
    EndEffectorCFEROS(const std::string& name, cnoid::Body* robot) {
      robot_ = robot;
      name_ = name;
      linkName_ = "";
      state_ = "NOT_CARED";
      contactPoint_ = std::make_shared<ContactPoint>();
      contactPoint_->name() = name_;
      ros::NodeHandle n;
      infoSub_ = n.subscribe(name_ + "/info", 1, &EndEffectorCFEROS::infoCallback, this);
      stateSub_ = n.subscribe(name_ + "/state", 1, &EndEffectorCFEROS::stateCallback, this);
      contactForcePub_ = n.advertise<geometry_msgs::WrenchStamped>(name_+"/force", 10);
    }
    std::string name() const { return name_; }
    std::string linkName() const { return linkName_; }
    std::string& linkName() {return linkName_; }
    std::shared_ptr<ContactPoint> contactPoint() const {return contactPoint_; }
    std::shared_ptr<ContactPoint>& contactPoint() {return contactPoint_; }
    std::string state() const {return state_; }
    std::string& state() {return state_; }
    ros::Publisher contactForcePub() const {return contactForcePub_; }
    ros::Publisher& contactForcePub() {return contactForcePub_; }
    void infoCallback(const multicontact_controller_msgs::EndEffectorInfo::ConstPtr& msg);
    void stateCallback(const std_msgs::String::ConstPtr& msg){ state_ = msg->data; }
  private:
    cnoid::Body* robot_;
    std::string name_;
    std::string linkName_;
    std::string state_;
    std::shared_ptr<ContactPoint> contactPoint_;

    std::map<std::string,cnoid::Link*> jointLinkMap_;
    ros::Subscriber infoSub_;
    ros::Subscriber stateSub_;
    ros::Publisher contactForcePub_;
  };

  class ContactForceEstimatorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;
    cnoid::Body* robot_;
    std::map<std::string,cnoid::Vector6> forceSensorOffsets_;

    std::map<std::string,std::shared_ptr<EndEffectorCFEROS> > endEffectors_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  };

};

#endif
