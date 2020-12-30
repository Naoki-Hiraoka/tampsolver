#ifndef PWT_CONTROLLER_ROS_H
#define PWT_CONTROLLER_ROS_H

#include <multicontact_controller/PWTController/PWTController.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/JointState.h>
#include <hrpsys_ros_bridge/MotorStates.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/MotorTemperatureState.h>
#include <multicontact_controller_msgs/PWTControllerConfig.h>
#include <multicontact_controller_msgs/CollisionArray.h>
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <pr2_controllers_msgs/JointTrajectoryControllerState.h>
#include <actionlib/client/simple_action_client.h>

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
      contactPoint_->state() = state_;

      ros::NodeHandle nh;
      forceSub_ = nh.subscribe(name_ + "/force_filtered", 1, &EndEffectorPWTCROS::forceCallback, this);
      refForceSub_ = nh.subscribe(name_ + "/ref_force", 1, &EndEffectorPWTCROS::refForceCallback, this);
      targetPoseSub_ = nh.subscribe(name_ + "/target_pose", 1, &EndEffectorPWTCROS::targetPoseCallback, this);
    }
    std::shared_ptr<ContactPointPWTC> contactPoint() const {return contactPoint_; }
    std::shared_ptr<ContactPointPWTC>& contactPoint() {return contactPoint_; }
    void onInfoUpdated() override;
    void onStateUpdated() override;

    void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
      if(this->isValid()){
        tf::wrenchMsgToEigen(msg->wrench,contactPoint_->F());
        contactPoint_->interaction()->F() = contactPoint_->F();
      }
    };
    void refForceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
      if(this->isValid()){
        tf::wrenchMsgToEigen(msg->wrench,contactPoint_->interaction()->F_ref());
      }
    };
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
      if(this->isValid()){
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(msg->pose,pose);
        contactPoint_->interaction()->T_ref() = pose;
      }
    };

    bool isValid() override{
      return endeffectorutils::EndEffectorClient::isValid() && contactPoint_->isValid();
    }

  private:
    cnoid::Body* robot_;
    std::shared_ptr<ContactPointPWTC> contactPoint_;

    ros::Subscriber forceSub_;
    ros::Subscriber refForceSub_;
    ros::Subscriber targetPoseSub_;
  };

  class PWTControllerROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void motorStatesCallback(const hrpsys_ros_bridge::MotorStates::ConstPtr& msg);
    void motorTemperatureStateCallback(const multicontact_controller_msgs::MotorTemperatureState::ConstPtr& msg);
    void controllerStateCallback(const pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr& msg);
    void selfCollisionCallback(const multicontact_controller_msgs::CollisionArray::ConstPtr& msg);
    void pclCollisionCallback(const multicontact_controller_msgs::CollisionArray::ConstPtr& msg);
    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
    void configCallback(multicontact_controller_msgs::PWTControllerConfig& config, int32_t level);
    multicontact_controller_msgs::PWTControllerConfig getCurrentConfig();

    bool isEnabled_;

    cnoid::Body* robot_;
    std::map<std::string, std::shared_ptr<EndEffectorPWTCROS> > endEffectors_;
    std::map<std::string, std::shared_ptr<cnoidbodyutils::JointInfo> > jointInfoMap_;
    std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> > jointInfos_;
    std::vector<std::shared_ptr<cnoidbodyutils::Collision> > selfCollisions_;
    std::vector<std::shared_ptr<cnoidbodyutils::Collision> > pclCollisions_;
    std::shared_ptr<PWTController> PWTController_;

    pr2_controllers_msgs::JointTrajectoryControllerState::ConstPtr controllerState_;

    // param
    double rate_;
    std::shared_ptr<ros::Rate> rosRate_;
    double smooth_ratio_;
  };

};

#endif
