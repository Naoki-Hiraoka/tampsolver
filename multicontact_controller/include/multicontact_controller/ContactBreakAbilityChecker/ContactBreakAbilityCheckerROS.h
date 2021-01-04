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
  class EndEffectorCBACROS : public EndEffectorPCLCDROS {
  public:
    EndEffectorCBACROS(const std::string& name, cnoid::Body* robot):
      EndEffectorPCLCDROS::EndEffectorPCLCDROS(name,robot)
    {
      contactPoint_ = std::make_shared<ContactPointCBAC>();
      drawRobot_ = robot_->clone();
    }

    void onInfoUpdated() override;
    void onStateUpdated() override;

    std::shared_ptr<ContactPointCBAC> contactPoint() const { return std::dynamic_pointer_cast<ContactPointCBAC>(contactPoint_);}

    std::vector<cnoid::SgNodePtr> getDrawOnObjects() override;
    cnoid::Body* drawRobot() {return drawRobot_;}
  private:
    //for visualize
    cnoid::SgLineSetPtr lines;
    cnoid::Body* drawRobot_;
  };

  class ContactBreakAbilityCheckerROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;

    void draw();
  private:
    bool isEnabled_;

    cnoid::Body* robot_;

    std::map<std::string, std::shared_ptr<EndEffectorCBACROS> > endEffectors_;
    std::map<std::string, std::shared_ptr<cnoidbodyutils::JointInfo> > jointInfoMap_;
    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> > jointInfos_;

    std::shared_ptr<PCLCollisionDetector> pclCollisionDetector_;
    std::shared_ptr<ContactBreakAbilityChecker> contactBreakAbilityChecker_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);

  };

};

#endif
