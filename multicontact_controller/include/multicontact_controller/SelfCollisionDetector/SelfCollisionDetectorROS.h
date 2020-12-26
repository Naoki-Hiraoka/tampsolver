#ifndef SELF_COLLISION_DETECTOR_ROS_H
#define SELF_COLLISION_DETECTOR_ROS_H

#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetector.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <sensor_msgs/JointState.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller {

  class SelfCollisionDetectorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;
    cnoid::Body* robot_;

    std::shared_ptr<SelfCollisionDetector> selfCollisionDetector_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  };

};

#endif
