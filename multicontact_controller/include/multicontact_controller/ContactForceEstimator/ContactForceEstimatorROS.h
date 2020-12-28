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

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller {
  class EndEffectorCFEROS: public endeffectorutils::EndEffectorClient {
  public:
    EndEffectorCFEROS(const std::string& name, cnoid::Body* robot)
      : endeffectorutils::EndEffectorClient::EndEffectorClient(name)
    {
      robot_ = robot;
      linkName_ = "";
      contactPoint_ = std::make_shared<ContactPointCFE>();
      contactPoint_->name() = name_;
      ros::NodeHandle n;
      contactForcePub_ = n.advertise<geometry_msgs::WrenchStamped>(name_+"/force", 10);
    }
    std::shared_ptr<ContactPointCFE> contactPoint() const {return contactPoint_; }
    std::shared_ptr<ContactPointCFE>& contactPoint() {return contactPoint_; }
    ros::Publisher contactForcePub() const {return contactForcePub_; }
    ros::Publisher& contactForcePub() {return contactForcePub_; }
    void onInfoUpdated() override;
  private:
    cnoid::Body* robot_;
    std::string linkName_;
    std::shared_ptr<ContactPointCFE> contactPoint_;

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
