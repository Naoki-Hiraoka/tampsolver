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

  {
    std::string contact_ns = ns + "/contact";
    if(!n.hasParam(contact_ns+"/type")){
      ROS_WARN("rosparam %s not found",(ns+"/type").c_str());
      endEffectorInfo->contact.type="SURFACE";
    }else{
      n.getParam(contact_ns+"/type",endEffectorInfo->contact.type);
    }

    if(!n.hasParam(contact_ns+"/vertices")){
      ROS_WARN("rosparam %s not found",(ns+"/vertices").c_str());
      endEffectorInfo->contact.vertices.points.clear();
      geometry_msgs::Point32 p1; p1.x = 0.05; p1.y = 0.05; p1.z = 0;
      endEffectorInfo->contact.vertices.points.push_back(p1);
      geometry_msgs::Point32 p2; p2.x = -0.05; p2.y = 0.05; p2.z = 0;
      endEffectorInfo->contact.vertices.points.push_back(p2);
      geometry_msgs::Point32 p3; p3.x = -0.05; p3.y = -0.05; p3.z = 0;
      endEffectorInfo->contact.vertices.points.push_back(p3);
      geometry_msgs::Point32 p4; p4.x = 0.05; p4.y = -0.05; p4.z = 0;
      endEffectorInfo->contact.vertices.points.push_back(p4);
    }else{
      std::vector<double> param;
      n.getParam(contact_ns+"/vertices",param);
      if(param.size() % 3 !=0) ROS_WARN("rosparam %s size mismatch %lu",(ns+"/vertices").c_str(), param.size());
      endEffectorInfo->contact.vertices.points.clear();
      for(size_t i=0;i<param.size() / 3; i++){
        geometry_msgs::Point32 p; p.x = param[i*3+0]; p.y = param[i*3+1]; p.z = param[i*3+2];
        endEffectorInfo->contact.vertices.points.push_back(p);
      }
    }

    if(!n.hasParam(contact_ns+"/mu_trans")){
      ROS_WARN("rosparam %s not found",(ns+"/mu_trans").c_str());
      endEffectorInfo->contact.mu_trans=0.1;
    }else{
      n.getParam(contact_ns+"/mu_trans",endEffectorInfo->contact.mu_trans);
    }

    if(!n.hasParam(contact_ns+"/mu_rot")){
      ROS_WARN("rosparam %s not found",(ns+"/mu_rot").c_str());
      endEffectorInfo->contact.mu_rot=0.1;
    }else{
      n.getParam(contact_ns+"/mu_rot",endEffectorInfo->contact.mu_rot);
    }

    if(!n.hasParam(contact_ns+"/max_fz")){
      ROS_WARN("rosparam %s not found",(ns+"/max_fz").c_str());
      endEffectorInfo->contact.max_fz=200.0;
    }else{
      n.getParam(contact_ns+"/max_fz",endEffectorInfo->contact.max_fz);
    }

    if(!n.hasParam(contact_ns+"/min_fz")){
      ROS_WARN("rosparam %s not found",(ns+"/min_fz").c_str());
      endEffectorInfo->contact.min_fz=50.0;
    }else{
      n.getParam(contact_ns+"/min_fz",endEffectorInfo->contact.min_fz);
    }

    if(!n.hasParam(contact_ns+"/contact_decision_threshold1")){
      ROS_WARN("rosparam %s not found",(ns+"/contact_decision_threshold1").c_str());
      endEffectorInfo->contact.contact_decision_threshold1=30.0;
    }else{
      n.getParam(contact_ns+"/contact_decision_threshold1",endEffectorInfo->contact.contact_decision_threshold1);
    }

    if(!n.hasParam(contact_ns+"/contact_decision_threshold2")){
      ROS_WARN("rosparam %s not found",(ns+"/contact_decision_threshold2").c_str());
      endEffectorInfo->contact.contact_decision_threshold2=50.0;
    }else{
      n.getParam(contact_ns+"/contact_decision_threshold2",endEffectorInfo->contact.contact_decision_threshold2);
    }

  }

  return true;
}

int main(int argc, char** argv){
  ros::init(argc,argv,"end_effector_info_publisher");
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

      res.success = true;
      return true;});

  ros::ServiceServer removeEndEffectorService
    = nl.advertiseService<multicontact_controller_msgs::SetString::Request, multicontact_controller_msgs::SetString::Response>
    ("remove_end_effector",
     [&](multicontact_controller_msgs::SetString::Request  &req,
         multicontact_controller_msgs::SetString::Response &res){
      if(endEffectorInfos.find(req.name) == endEffectorInfos.end()){
        res.success = true;
        return true;
      }
      endEffectorInfoPubs.erase(req.name);
      multicontact_controller_msgs::StringArray msg;
      for(std::map<std::string,std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> >::iterator it=endEffectorInfos.begin();it!=endEffectorInfos.end();it++){
        msg.strings.push_back(it->first);
      }
      endEffectorsPub.publish(msg);

      res.success = true;
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
