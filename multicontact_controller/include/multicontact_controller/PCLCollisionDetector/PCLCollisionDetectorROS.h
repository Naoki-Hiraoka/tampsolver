#ifndef PCL_COLLISION_DETECTOR_ROS_H
#define PCL_COLLISION_DETECTOR_ROS_H

#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetector.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <std_srvs/SetBool.h>
#include <ros/ros.h>

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller {

  class PCLCollisionDetectorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;
    cnoid::Body* robot_;

    std::shared_ptr<PCLCollisionDetector> pclCollisionDetector_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  };

};

#endif
