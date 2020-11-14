#ifndef CONTACT_FORCE_ESTIMATOR_ROS_H
#define CONTACT_FORCE_ESTIMATOR_ROS_H

#include <multicontact_controller/ContactForceEstimator/ContactForceEstimator.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>
#include <sensor_msgs/JointState.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/WrenchStamped.h>
#include <urdf/model.h>
#include <multicontact_controller_msgs/ContactPoints.h>

namespace multicontact_controller {

  class ContactForceEstimatorROS : public choreonoid_cpp::ChoreonoidCpp {
  public:
    virtual void main(int argc, char** argv) override;
  private:
    cnoid::Body* robot_;
    urdf::Model model_;
    std::vector<std::shared_ptr<ContactPoint> > contactPoints_;

    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg);
    void contactPointsCallback(const multicontact_controller_msgs::ContactPoints::ConstPtr& msg);
  };

};

#endif
