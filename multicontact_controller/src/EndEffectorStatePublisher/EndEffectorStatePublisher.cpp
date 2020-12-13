#include <ros/ros.h>
#include <multicontact_controller_msgs/EndEffectorInfoArray.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>

class EndEffectorState {
public:
  EndEffectorState() {
  }
  std::string name() const { return name_; }
  std::string& name() { return name_; }
  std::string linkName() const { return linkName_; }
  std::string& linkName() {return linkName_; }
  Eigen::Affine3d localpos() const {return localpos_; }
  Eigen::Affine3d& localpos() {return localpos_; }
  int32_t state() const {return state_; }
  int32_t& state() {return state_; }

  bool loadParam(ros::NodeHandle& n, const std::string& ns) {
    if(!n.hasParam(ns+"/name")){
      ROS_WARN("rosparam %s not found",(ns+"/name").c_str());
      return false;
    }
    n.getParam(ns+"/name",name_);

    if(!n.hasParam(ns+"/link")){
      ROS_WARN("rosparam %s not found",(ns+"/link").c_str());
      return false;
    }
    n.getParam(ns+"/link",linkName_);

    if(!n.hasParam(ns+"/localpos")){
      localpos_.setIdentity();
    }else{
      std::vector<double> params;
      n.getParam(ns+"/localpos",params);
      if(params.size()!=7){
        ROS_WARN("size of %s != 7",(ns+"/localpos").c_str());
        return false;
      }else{
        Eigen::Vector3d pos;
        for(size_t j=0;j<3;j++){
          pos[j]=params[j];
        }
        Eigen::Vector3d axis;
        for(size_t j=0;j<3;j++){
          axis[j]=params[3+j];
        }
        double angle = params[6];
        localpos_.translation() = pos;
        localpos_.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(angle,axis.normalized()));
      }

      if(!n.hasParam(ns+"/initialstate")){
        state_ = multicontact_controller_msgs::EndEffectorInfo::NOT_CARED;
      }else{
        std::string param;
        n.getParam(ns+"/initialstate",param);
        if(param=="NOT_CARED") state_ = multicontact_controller_msgs::EndEffectorInfo::NOT_CARED;
        else if(param=="AIR") state_ = multicontact_controller_msgs::EndEffectorInfo::AIR;
        else if(param=="AIR_NEAR_CONTACT") state_ = multicontact_controller_msgs::EndEffectorInfo::AIR_NEAR_CONTACT;
        else if(param=="TOWARD_BREAK_CONTACT") state_ = multicontact_controller_msgs::EndEffectorInfo::TOWARD_BREAK_CONTACT;
        else if(param=="TOWARD_MAKE_CONTACT") state_ = multicontact_controller_msgs::EndEffectorInfo::TOWARD_MAKE_CONTACT;
        else if(param=="CONTACT") state_ = multicontact_controller_msgs::EndEffectorInfo::CONTACT;
        else{
          ROS_WARN("%s is not supported",param.c_str());
          return false;
        }
      }
    }

    return true;
  }

  multicontact_controller_msgs::EndEffectorInfo toMsg() {
    multicontact_controller_msgs::EndEffectorInfo msg;
    msg.header.frame_id = linkName_;
    msg.name = name_;
    tf::transformEigenToMsg(localpos_,msg.transform);
    msg.state = state_;
    return msg;
  }
private:
  std::string name_;
  std::string linkName_;
  Eigen::Affine3d localpos_;
  int32_t state_;//multicontact_controller_msgs::EndEffectorMsg::state
};


int main(int argc, char** argv){
  ros::init(argc,argv,"end_effector_state_pubisher");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  std::vector<std::string> end_effectors;
  n.getParam("end_effectors",end_effectors);

  std::map<std::string,std::shared_ptr<EndEffectorState> > endEffectorStates;

  for(size_t i=0;i<end_effectors.size();i++){
    std::string ns = "end_effector_config/" + end_effectors[i];
    std::shared_ptr<EndEffectorState> endEffectorState = std::make_shared<EndEffectorState>();
    if(endEffectorState->loadParam(n,ns)){
      endEffectorStates[endEffectorState->name()] = endEffectorState;
    }
  }

  ros::Publisher endEffectorStateArrayPub = n.advertise<multicontact_controller_msgs::EndEffectorInfoArray>("end_effector_states", 1000);

  tf2_ros::TransformBroadcaster br;

  int rate;
  nl.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  unsigned int seq = 0;
  while (ros::ok()) {
    // spin
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    // publish EffectorStateArray
    multicontact_controller_msgs::EndEffectorInfoArray msg;
    for(std::map<std::string,std::shared_ptr<EndEffectorState> >::iterator it=endEffectorStates.begin();it!=endEffectorStates.end();it++){
      multicontact_controller_msgs::EndEffectorInfo state = it->second->toMsg();
      state.header.stamp = now;
      state.header.seq = seq;
      msg.endeffectorstates.push_back(std::move(state));
    }
    endEffectorStateArrayPub.publish(msg);

    // publish tf for EndEffectors
    for(std::map<std::string,std::shared_ptr<EndEffectorState> >::iterator it=endEffectorStates.begin();it!=endEffectorStates.end();it++){
      geometry_msgs::TransformStamped msg;
      msg.header.stamp = now;
      msg.header.seq = seq;
      msg.header.frame_id = it->second->linkName();
      msg.child_frame_id = it->second->name();
      tf::transformEigenToMsg(it->second->localpos(),msg.transform);

      br.sendTransform(msg);
    }


    seq++;
    r.sleep();
  }

  return 0;
}
