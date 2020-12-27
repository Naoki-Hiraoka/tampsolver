#include <ros/ros.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/SetString.h>
#include <multicontact_controller_msgs/EndEffectorInfoConfig.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <dynamic_reconfigure/server.h>
#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

class EndEffectorInfoPublisher{
public:
  EndEffectorInfoPublisher(const std::string& name)
    : name_(name)
  {
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~"+name);
    endEffectorInfoPub_ = nh.advertise<multicontact_controller_msgs::EndEffectorInfo>(name_ + "/info", 100, true); //latch
    this->loadEndEffectorInfoParam(name_, endEffectorInfo_);
    endEffectorInfoPub_.publish(*endEffectorInfo_);
    configServer_ = std::make_shared<dynamic_reconfigure::Server<multicontact_controller_msgs::EndEffectorInfoConfig> >(pnh);
    multicontact_controller_msgs::EndEffectorInfoConfig config = this->getCurrentConfig();
    configServer_->setConfigDefault(config);
    configServer_->updateConfig(config);
    configServer_->setCallback(std::bind(&EndEffectorInfoPublisher::configCallback, this, std::placeholders::_1, std::placeholders::_2));//この中でcallbackが呼ばれるので、その前にupdateConfigを呼ぶ必要がある
  }

  std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> endEffectorInfo() { return endEffectorInfo_;}
protected:
  bool loadEndEffectorInfoParam(const std::string& name, std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo>& endEffectorInfo){
    if(!endEffectorInfo) endEffectorInfo = std::make_shared<multicontact_controller_msgs::EndEffectorInfo>();
    ros::NodeHandle nh;
    std::string ns = "end_effector_config/" + name;

    if(!nh.hasParam(ns+"/link")){
      ROS_WARN("rosparam %s not found",(ns+"/link").c_str());
      return false;
    }
    nh.getParam(ns+"/link",endEffectorInfo->header.frame_id);

    if(!nh.hasParam(ns+"/localpos")){
      Eigen::Affine3d trans;
      trans.setIdentity();
      tf::transformEigenToMsg(trans, endEffectorInfo->transform);
    }else{
      std::vector<double> params;
      nh.getParam(ns+"/localpos",params);
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
      if(!nh.hasParam(contact_ns+"/type")){
        ROS_WARN("rosparam %s not found",(ns+"/type").c_str());
        endEffectorInfo->contact.type="SURFACE";
      }else{
        nh.getParam(contact_ns+"/type",endEffectorInfo->contact.type);
      }

      if(!nh.hasParam(contact_ns+"/vertices")){
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
        nh.getParam(contact_ns+"/vertices",param);
        if(param.size() % 3 !=0) ROS_WARN("rosparam %s size mismatch %lu",(ns+"/vertices").c_str(), param.size());
        endEffectorInfo->contact.vertices.points.clear();
        for(size_t i=0;i<param.size() / 3; i++){
          geometry_msgs::Point32 p; p.x = param[i*3+0]; p.y = param[i*3+1]; p.z = param[i*3+2];
          endEffectorInfo->contact.vertices.points.push_back(p);
        }
      }

      if(!nh.hasParam(contact_ns+"/mu_trans")){
        ROS_WARN("rosparam %s not found",(ns+"/mu_trans").c_str());
        endEffectorInfo->contact.mu_trans=0.1;
      }else{
        nh.getParam(contact_ns+"/mu_trans",endEffectorInfo->contact.mu_trans);
      }

      if(!nh.hasParam(contact_ns+"/max_fz")){
        ROS_WARN("rosparam %s not found",(ns+"/max_fz").c_str());
        endEffectorInfo->contact.max_fz=200.0;
      }else{
        nh.getParam(contact_ns+"/max_fz",endEffectorInfo->contact.max_fz);
      }

      if(!nh.hasParam(contact_ns+"/min_fz")){
        ROS_WARN("rosparam %s not found",(ns+"/min_fz").c_str());
        endEffectorInfo->contact.min_fz=50.0;
      }else{
        nh.getParam(contact_ns+"/min_fz",endEffectorInfo->contact.min_fz);
      }

      if(!nh.hasParam(contact_ns+"/contact_decision_threshold1")){
        ROS_WARN("rosparam %s not found",(ns+"/contact_decision_threshold1").c_str());
        endEffectorInfo->contact.contact_decision_threshold1=30.0;
      }else{
        nh.getParam(contact_ns+"/contact_decision_threshold1",endEffectorInfo->contact.contact_decision_threshold1);
      }

      if(!nh.hasParam(contact_ns+"/contact_decision_threshold2")){
        ROS_WARN("rosparam %s not found",(ns+"/contact_decision_threshold2").c_str());
        endEffectorInfo->contact.contact_decision_threshold2=50.0;
      }else{
        nh.getParam(contact_ns+"/contact_decision_threshold2",endEffectorInfo->contact.contact_decision_threshold2);
      }

    }

    {
      std::string interaction_ns = ns + "/interaction";

      if(!nh.hasParam(interaction_ns+"/force_gain")){
        ROS_WARN("rosparam %s not found",(ns+"/force_gain").c_str());
        endEffectorInfo->interaction.force_gain.x = 1.0;
        endEffectorInfo->interaction.force_gain.y = 1.0;
        endEffectorInfo->interaction.force_gain.z = 1.0;
      }else{
        std::vector<double> param;
        nh.getParam(interaction_ns+"/force_gain",param);
        if(param.size() !=3){
          ROS_WARN("rosparam %s size mismatch %lu",(ns+"/force_gain").c_str(), param.size());
          endEffectorInfo->interaction.force_gain.x = 1.0;
          endEffectorInfo->interaction.force_gain.y = 1.0;
          endEffectorInfo->interaction.force_gain.z = 1.0;
        }else{
          endEffectorInfo->interaction.force_gain.x = param[0];
          endEffectorInfo->interaction.force_gain.y = param[1];
          endEffectorInfo->interaction.force_gain.z = param[2];
        }
      }

      if(!nh.hasParam(interaction_ns+"/M_p")){
        ROS_WARN("rosparam %s not found",(ns+"/M_p").c_str());
        endEffectorInfo->interaction.M_p=100.0;
      }else{
        nh.getParam(interaction_ns+"/M_p",endEffectorInfo->interaction.M_p);
      }
      if(!nh.hasParam(interaction_ns+"/D_p")){
        ROS_WARN("rosparam %s not found",(ns+"/D_p").c_str());
        endEffectorInfo->interaction.D_p=2000.0;
      }else{
        nh.getParam(interaction_ns+"/D_p",endEffectorInfo->interaction.D_p);
      }
      if(!nh.hasParam(interaction_ns+"/K_p")){
        ROS_WARN("rosparam %s not found",(ns+"/K_p").c_str());
        endEffectorInfo->interaction.K_p=2000.0;
      }else{
        nh.getParam(interaction_ns+"/K_p",endEffectorInfo->interaction.K_p);
      }
      if(!nh.hasParam(interaction_ns+"/M_r")){
        ROS_WARN("rosparam %s not found",(ns+"/M_r").c_str());
        endEffectorInfo->interaction.M_r=50.0;
      }else{
        nh.getParam(interaction_ns+"/M_r",endEffectorInfo->interaction.M_r);
      }
      if(!nh.hasParam(interaction_ns+"/D_r")){
        ROS_WARN("rosparam %s not found",(ns+"/D_r").c_str());
        endEffectorInfo->interaction.D_r=2000.0;
      }else{
        nh.getParam(interaction_ns+"/D_r",endEffectorInfo->interaction.D_r);
      }
      if(!nh.hasParam(interaction_ns+"/K_r")){
        ROS_WARN("rosparam %s not found",(ns+"/K_r").c_str());
        endEffectorInfo->interaction.K_r=2000.0;
      }else{
        nh.getParam(interaction_ns+"/K_r",endEffectorInfo->interaction.K_r);
      }
    }

    if(!nh.hasParam(ns+"/allow_collision_links")){
      ROS_WARN("rosparam %s not found",(ns+"/allow_collision_links").c_str());
      endEffectorInfo->allow_collision_links.clear();
    }else{
      nh.getParam(ns+"/allow_collision_links",endEffectorInfo->allow_collision_links);
    }

    return true;
  }

  void configCallback(multicontact_controller_msgs::EndEffectorInfoConfig& config, int32_t level){
    endEffectorInfo_->contact.type = config.type;
    endEffectorInfo_->contact.mu_trans = config.mu_trans;
    endEffectorInfo_->contact.max_fz = config.max_fz;
    endEffectorInfo_->contact.min_fz = config.min_fz;
    endEffectorInfo_->contact.contact_decision_threshold1 = config.contact_decision_threshold1;
    endEffectorInfo_->contact.contact_decision_threshold2 = config.contact_decision_threshold2;
    endEffectorInfo_->interaction.M_p = config.M_p;
    endEffectorInfo_->interaction.D_p = config.D_p;
    endEffectorInfo_->interaction.K_p = config.K_p;
    endEffectorInfo_->interaction.M_r = config.M_r;
    endEffectorInfo_->interaction.D_r = config.D_r;
    endEffectorInfo_->interaction.K_r = config.K_r;
    endEffectorInfo_->interaction.force_gain.x = config.force_gain_x;
    endEffectorInfo_->interaction.force_gain.y = config.force_gain_y;
    endEffectorInfo_->interaction.force_gain.z = config.force_gain_z;
    endEffectorInfo_->interaction.moment_gain.x = config.moment_gain_x;
    endEffectorInfo_->interaction.moment_gain.y = config.moment_gain_y;
    endEffectorInfo_->interaction.moment_gain.z = config.moment_gain_z;
    multicontact_controller::endeffectorutils::stringToVector(config.allow_collision_links,endEffectorInfo_->allow_collision_links);

    endEffectorInfoPub_.publish(*endEffectorInfo_);
  }

  multicontact_controller_msgs::EndEffectorInfoConfig getCurrentConfig(){
    multicontact_controller_msgs::EndEffectorInfoConfig config;
    config.type = endEffectorInfo_->contact.type;
    config.mu_trans = endEffectorInfo_->contact.mu_trans;
    config.max_fz = endEffectorInfo_->contact.max_fz;
    config.min_fz = endEffectorInfo_->contact.min_fz;
    config.contact_decision_threshold1 = endEffectorInfo_->contact.contact_decision_threshold1;
    config.contact_decision_threshold2 = endEffectorInfo_->contact.contact_decision_threshold2;
    config.M_p = endEffectorInfo_->interaction.M_p;
    config.D_p = endEffectorInfo_->interaction.D_p;
    config.K_p = endEffectorInfo_->interaction.K_p;
    config.M_r = endEffectorInfo_->interaction.M_r;
    config.D_r = endEffectorInfo_->interaction.D_r;
    config.K_r  =endEffectorInfo_->interaction.K_r;
    config.force_gain_x = endEffectorInfo_->interaction.force_gain.x;
    config.force_gain_y = endEffectorInfo_->interaction.force_gain.y;
    config.force_gain_z = endEffectorInfo_->interaction.force_gain.z;
    config.moment_gain_x = endEffectorInfo_->interaction.moment_gain.x;
    config.moment_gain_y = endEffectorInfo_->interaction.moment_gain.y;
    config.moment_gain_z = endEffectorInfo_->interaction.moment_gain.z;
    multicontact_controller::endeffectorutils::vectorToString(endEffectorInfo_->allow_collision_links,config.allow_collision_links);

    return config;
  }

private:
  std::string name_;
  std::shared_ptr<multicontact_controller_msgs::EndEffectorInfo> endEffectorInfo_;
  ros::Publisher endEffectorInfoPub_;
  std::shared_ptr<dynamic_reconfigure::Server<multicontact_controller_msgs::EndEffectorInfoConfig> > configServer_;
};


int main(int argc, char** argv){
  ros::init(argc,argv,"end_effector_info_publisher");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  ros::Publisher endEffectorsPub = n.advertise<multicontact_controller_msgs::StringArray>("end_effectors", 100, true); //latch
  std::map<std::string, std::shared_ptr<EndEffectorInfoPublisher> > endEffectorInfoPublishers;

  ros::ServiceServer addEndEffectorService
    = nl.advertiseService<multicontact_controller_msgs::SetString::Request, multicontact_controller_msgs::SetString::Response>
    ("add_end_effector",
     [&](multicontact_controller_msgs::SetString::Request  &req,
         multicontact_controller_msgs::SetString::Response &res){
      if(endEffectorInfoPublishers.find(req.name) == endEffectorInfoPublishers.end()){
        endEffectorInfoPublishers[req.name] = std::make_shared<EndEffectorInfoPublisher>(req.name);
      }

      multicontact_controller_msgs::StringArray msg;
      for(std::map<std::string,std::shared_ptr<EndEffectorInfoPublisher> >::iterator it=endEffectorInfoPublishers.begin();it!=endEffectorInfoPublishers.end();it++){
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
      if(endEffectorInfoPublishers.find(req.name) == endEffectorInfoPublishers.end()){
        res.success = true;
        return true;
      }
      endEffectorInfoPublishers.erase(req.name);
      multicontact_controller_msgs::StringArray msg;
      for(std::map<std::string,std::shared_ptr<EndEffectorInfoPublisher> >::iterator it=endEffectorInfoPublishers.begin();it!=endEffectorInfoPublishers.end();it++){
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
    for(std::map<std::string,std::shared_ptr<EndEffectorInfoPublisher> >::iterator it=endEffectorInfoPublishers.begin();it!=endEffectorInfoPublishers.end();it++){
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = now;
      msg.header.seq = seq;
      msg.header.frame_id = it->second->endEffectorInfo()->header.frame_id;
      msg.child_frame_id = it->first;
      msg.transform = it->second->endEffectorInfo()->transform;

      br.sendTransform(msg);
    }

    seq++;
    r.sleep();
  }

  return 0;
}
