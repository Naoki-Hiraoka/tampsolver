#include <ros/ros.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/SetString.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

bool loadEndEffectorInfoParam(ros::NodeHandle& n, std::string& name, std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> endEffectorInfo){
  std::string ns = "end_effector_config/" + name;

  if(!n.hasParam(ns+"/link")){
    ROS_WARN("rosparam %s not found",(ns+"/link").c_str());
      return false;
  }
  n.getParam(ns+"/link",endEffectorInfo->header.frame_id);

  if(!n.hasParam(ns+"/localpos")){
    Eigen::Affine3d trans;
    trans.setIdentity();
    tf::transformEigenToMsg(trans, endEffectorInfo->transform);
  }else{
      std::vector<double> params;
      n.getParam(ns+"/localpos",params);
      if(params.size()!=7){
        ROS_WARN("size of %s != 7",(ns+"/localpos").c_str());
        return false;
      }else{
        Eigen::Affine3d trans;
        for(size_t j=0;j<3;j++){
          trans.translation()[j]=params[j];
        }
        trans.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(params[6],Eigen::Vector3d(params[3],params[4],params[5]).normalized()));
        tf::transformEigenToMsg(trans, endEffectorInfo->transform);
      }
  }

  return true;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"end_effector_info_pubisher");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  ros::Publisher endEffectorsPub = n.advertise<multicontact_controller_msgs::StringArray>("end_effectors", 100, true); //latch
  std::map<std::string, std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> > endEffectorInfos;
  std::map<std::string, ros::Publisher > endEffectorInfoPubs;

  ros::ServiceServer addEndEffectorService
    = nl.advertiseService<multicontact_controller_msgs::SetString::Request, multicontact_controller_msgs::SetString::Response>
    ("add_end_effector",
     [&](multicontact_controller_msgs::SetString::Request  &req,
         multicontact_controller_msgs::SetString::Response &res){
      if(endEffectorInfos.find(req.name) == endEffectorInfos.end()){
        endEffectorInfos[req.name] = std::make_shared<multicontact_controller_msgs::EndEffectorInfo>();
      }
      if(endEffectorInfoPubs.find(req.name) == endEffectorInfoPubs.end()){
        endEffectorInfoPubs[req.name] = n.advertise<multicontact_controller_msgs::EndEffectorInfo>(req.name + "/info", 100, true); //latch
      }
      loadEndEffectorInfoParam(n,req.name,endEffectorInfos[req.name]);

      endEffectorInfoPubs[req.name].publish(*endEffectorInfos[req.name]);
      multicontact_controller_msgs::StringArray msg;
      for(std::map<std::string,std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> >::iterator it=endEffectorInfos.begin();it!=endEffectorInfos.end();it++){
        msg.strings.push_back(it->first);
      }
      endEffectorsPub.publish(msg);

      return true;});

  tf2_ros::TransformBroadcaster br;

  int rate;
  nl.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  unsigned int seq = 0;
  while (ros::ok()) {
    // spin
    ros::spinOnce();

    ros::Time now = ros::Time::now();

    // publish tf for EndEffectors
    for(std::map<std::string,std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> >::iterator it=endEffectorInfos.begin();it!=endEffectorInfos.end();it++){
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = now;
      msg.header.seq = seq;
      msg.header.frame_id = it->second->header.frame_id;
      msg.child_frame_id = it->first;
      msg.transform = it->second->transform;

      br.sendTransform(msg);
    }

    seq++;
    r.sleep();
  }

  return 0;
}
