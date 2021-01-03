#ifndef CONTACT_BREAKABILITY_CHECKER_ROS_H
#define CONTACT_BREAKABILITY_CHECKER_ROS_H

#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityChecker.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/WrenchStamped.h>
#include <nav_msgs/Odometry.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/CollisionArray.h>
#include <ros/ros.h>

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller {
  class ContactBreakAbilityCheckerROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;

    cnoid::Body* robot_;
  };

};

#endif
