#ifndef PWT_CONTROLLER_ROS_H
#define PWT_CONTROLLER_ROS_H

#include <multicontact_controller/PWTController/PWTController.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/SetTransformStamped.h>
#include <ros/ros.h>

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller {
  class EndEffectorPWTCROS: public endeffectorutils::EndEffectorClient {
  public:
    EndEffectorPWTCROS(const std::string& name, cnoid::Body* robot)
      : endeffectorutils::EndEffectorClient::EndEffectorClient(name)
    {
      robot_ = robot;
      contactPoint_ = std::make_shared<ContactPointPWTC>();
      contactPoint_->name() = name_;
      contactPoint_->state() = "NOT_CARED";
    }
    std::shared_ptr<ContactPointPWTC> contactPoint() const {return contactPoint_; }
    std::shared_ptr<ContactPointPWTC>& contactPoint() {return contactPoint_; }
    void onInfoUpdated() override;
    void onStateUpdated() override;
  private:
    cnoid::Body* robot_;
    std::shared_ptr<ContactPointPWTC> contactPoint_;
  };

  class PWTControllerROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    bool setBodyTransformCallback(multicontact_controller_msgs::SetTransformStamped::Request& request, multicontact_controller_msgs::SetTransformStamped::Response& response);

    bool isEnabled_;

    cnoid::Body* robot_;
    std::map<std::string, std::shared_ptr<EndEffectorPWTCROS> > endEffectors_;
    //PWTController multiContactFootCoords_;
  };

};

#endif
