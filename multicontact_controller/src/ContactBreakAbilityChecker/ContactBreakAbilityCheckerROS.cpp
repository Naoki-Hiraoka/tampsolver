#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROS.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

namespace multicontact_controller {
  void ContactBreakAbilityCheckerROS::main(int argc, char** argv) {

    ros::init(argc,argv,"contact_breakability_checker");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    robot_ = cnoidbodyutils::loadBodyFromParam("/vrml_file");
    if(!robot_){
      ROS_ERROR("Failed to load robot from '/vrml_file'");
      return;
    }
    objects(robot_);
  }

};
