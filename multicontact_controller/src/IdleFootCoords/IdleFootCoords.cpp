#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/TransformStamped.h>
#include <multicontact_controller_msgs/SetTransformStamped.h>

int main(int argc, char** argv){
  ros::init(argc,argv,"idle_footcoords");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");

  tf2_ros::TransformBroadcaster br;

  bool isActive;
  pn.param("start_enabled",isActive,false);
  ros::ServiceServer enableService
    = pn.advertiseService<std_srvs::SetBool::Request ,std_srvs::SetBool::Response>
    ("enable",
     [&](std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
      if(isActive == request.data){
        ROS_INFO("[IdleFootCoords::enableService] Already %s",(isActive ? "Enabled" : "Disabled"));
        response.success = true;
        response.message = "";
        return true;
      } else {
        isActive = request.data;
        ROS_INFO("[IdleFootCoords::enableService] %s",(isActive ? "Enabled" : "Disabled"));
        response.success = true;
        response.message = "";
        return true;
      }
     });

  geometry_msgs::TransformStamped bodyTransformStamped;
  bodyTransformStamped.header.frame_id = "odom";
  bodyTransformStamped.child_frame_id = "body";
  bodyTransformStamped.transform.translation.x = 0.0;
  bodyTransformStamped.transform.translation.y = 0.0;
  bodyTransformStamped.transform.translation.z = 0.0;
  bodyTransformStamped.transform.rotation.x = 0;
  bodyTransformStamped.transform.rotation.y = 0;
  bodyTransformStamped.transform.rotation.z = 1;
  bodyTransformStamped.transform.rotation.w = 0;
  ros::ServiceServer setBodyTransformService
    = pn.advertiseService<multicontact_controller_msgs::SetTransformStamped::Request,multicontact_controller_msgs::SetTransformStamped::Response>
    ("set_body_transform",
     [&](multicontact_controller_msgs::SetTransformStamped::Request& request, multicontact_controller_msgs::SetTransformStamped::Response& response){
      bodyTransformStamped = request.transform;
      response.success = true;
      response.message = "";
      return true;
     });

  geometry_msgs::TransformStamped baseFootPrintTransformStamped;
  baseFootPrintTransformStamped.header.frame_id = "body";
  baseFootPrintTransformStamped.child_frame_id = "base_footprint";
  baseFootPrintTransformStamped.transform.translation.x = 0.0;
  baseFootPrintTransformStamped.transform.translation.y = 0.0;
  baseFootPrintTransformStamped.transform.translation.z = 0.0;
  baseFootPrintTransformStamped.transform.rotation.x = 0;
  baseFootPrintTransformStamped.transform.rotation.y = 0;
  baseFootPrintTransformStamped.transform.rotation.z = 1;
  baseFootPrintTransformStamped.transform.rotation.w = 0;
  ros::ServiceServer setBaseFootPrintTransformService
    = pn.advertiseService<multicontact_controller_msgs::SetTransformStamped::Request,multicontact_controller_msgs::SetTransformStamped::Response>
    ("set_base_footprint_transform",
     [&](multicontact_controller_msgs::SetTransformStamped::Request& request, multicontact_controller_msgs::SetTransformStamped::Response& response){
      baseFootPrintTransformStamped = request.transform;
      response.success = true;
      response.message = "";
      return true;
     });

  int rate;
  pn.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  while (ros::ok()) {
    // spin
    ros::spinOnce();

    if(isActive){
      ros::Time now = ros::Time::now();

      // publish tf
      std::vector<geometry_msgs::TransformStamped> transformStampeds;

      bodyTransformStamped.header.stamp = now;
      transformStampeds.push_back(bodyTransformStamped);

      baseFootPrintTransformStamped.header.stamp = now;
      transformStampeds.push_back(baseFootPrintTransformStamped);

      br.sendTransform(transformStampeds);
    }

    r.sleep();
  }

  return 0;
}
