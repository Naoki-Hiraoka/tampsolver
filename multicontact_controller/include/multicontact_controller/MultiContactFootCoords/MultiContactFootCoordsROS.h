#ifndef MULTICONTACT_FOOTCOORDS_ROS_H
#define MULTICONTACT_FOOTCOORDS_ROS_H

#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoords.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <ros/ros.h>

namespace multicontact_controller {
  class MultiContactFootCoordsROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  };

};

#endif
