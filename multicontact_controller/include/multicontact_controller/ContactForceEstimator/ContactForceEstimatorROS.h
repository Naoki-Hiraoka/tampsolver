#ifndef CONTACT_FORCE_ESTIMATOR_ROS_H
#define CONTACT_FORCE_ESTIMATOR_ROS_H

#include <multicontact_controller/ContactForceEstimator/ContactForceEstimator.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <multicontact_controller_msgs/EndEffectorStateArray.h>
#include <ros/ros.h>

namespace multicontact_controller {
  class EndEffectorState {
  public:
    EndEffectorState(const std::string& name) {
      name_ = name;
      contactPoint_ = std::make_shared<ContactPoint>();
      ros::NodeHandle n;
      contactForcePub_ = n.advertise<geometry_msgs::WrenchStamped>(name_+"force", 1000);
    }
    std::string name() const { return name_; }
    std::string linkName() const { return linkName_; }
    std::string& linkName() {return linkName_; }
    std::shared_ptr<ContactPoint> contactPoint() const {return contactPoint_; }
    std::shared_ptr<ContactPoint>& contactPoint() {return contactPoint_; }
    int32_t state() const {return state_; }
    int32_t& state() {return state_; }
    ros::Publisher contactForcePub() const {return contactForcePub_; }
    ros::Publisher& contactForcePub() {return contactForcePub_; }
  private:
    std::string name_;
    std::string linkName_;
    int32_t state_;//multicontact_controller_msgs::EndEffectorState::state
    std::shared_ptr<ContactPoint> contactPoint_;

    ros::Publisher contactForcePub_;
  };

  class ContactForceEstimatorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    cnoid::Body* robot_;
    std::map<std::string,cnoid::Link*> jointLinkMap_;

    std::map<std::string,std::shared_ptr<EndEffectorState> > endEffectorStates_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void contactPointsCallback(const multicontact_controller_msgs::EndEffectorStateArray::ConstPtr& msg);
  };

};

#endif
