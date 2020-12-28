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
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {
  class EndEffectorPCLCDROS: public endeffectorutils::EndEffectorClient {
  public:
    EndEffectorPCLCDROS(const std::string& name, cnoid::Body* robot):
      endeffectorutils::EndEffectorClient::EndEffectorClient(name),
      robot_(robot),
      allowCollisionBoxFilter_(std::make_shared<pcl::CropBox<pcl::PointXYZ> >()),
      contactPoint_(std::make_shared<cnoidbodyutils::ContactPoint>())
    {
      allowCollisionBoxCenter_.setIdentity();
    }

    std::vector<cnoid::Link*> allowCollisionLinks();
    std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > allowCollisionBoxFilter() const {return allowCollisionBoxFilter_;}
    std::shared_ptr<pcl::CropBox<pcl::PointXYZ> >& allowCollisionBoxFilter() {return allowCollisionBoxFilter_;}
    cnoid::Position& allowCollisionBoxCenter() { return allowCollisionBoxCenter_;}
    cnoid::Position allowCollisionBoxCenter() const { return allowCollisionBoxCenter_;}

    std::shared_ptr<cnoidbodyutils::ContactPoint>& contactPoint() { return contactPoint_;}
    std::shared_ptr<cnoidbodyutils::ContactPoint> contactPoint() const { return contactPoint_;}

    void onInfoUpdated() override;

    bool isValid() override{
      return endeffectorutils::EndEffectorClient::isValid() && contactPoint_->isValid();
    }

    std::vector<cnoid::SgNodePtr> getDrawOnObjects();
  private:
    cnoid::Body* robot_;
    std::vector<cnoid::Link*> allowCollisionLinks_;
    std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > allowCollisionBoxFilter_;
    cnoid::Position allowCollisionBoxCenter_;

    std::shared_ptr<cnoidbodyutils::ContactPoint> contactPoint_;

    //for visualize
    cnoid::SgLineSetPtr lines;
  };


  class PCLCollisionDetectorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    bool isEnabled_;
    cnoid::Body* robot_;

    std::map<std::string, std::shared_ptr<EndEffectorPCLCDROS> > endEffectors_;

    std::shared_ptr<PCLCollisionDetector> pclCollisionDetector_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

    void endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg);
    bool enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res);
  };

};

#endif
