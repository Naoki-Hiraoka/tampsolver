#ifndef WALK_CONTROLLER_ROS_H
#define WALK_CONTROLLER_ROS_H

#include <multicontact_controller/WalkController/WalkController.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

namespace multicontact_controller {
  class WalkControllerROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool enableCallback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
    bool onEnable();
    bool onDisable();

    cnoid::Body* robot_;
    bool isActive_;

    ros::ServiceClient shGetCommandClient_;
    geometry_msgs::TransformStamped abcBaseTform_x;

    std::string message_;
  };

};

#endif
