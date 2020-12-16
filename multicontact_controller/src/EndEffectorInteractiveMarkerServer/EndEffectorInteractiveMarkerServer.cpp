#include <ros/ros.h>

#include <urdf/model.h>
#include <tf/transform_listener.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/PoseStamped.h>

#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <multicontact_controller_msgs/StringArray.h>

#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

class EndEffectorInteractiveMarkerController: public multicontact_controller::endeffectorutils::EndEffectorClient {
public:
  EndEffectorInteractiveMarkerController(const std::string& name,
                                         std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer,
                                         std::shared_ptr<tf::TransformListener> tfListener,
                                         std::shared_ptr<urdf::Model> robot_model)
    : multicontact_controller::endeffectorutils::EndEffectorClient::EndEffectorClient(name)
  {
    interactiveMarkerServer_ = interactiveMarkerServer;
    tfListener_ = tfListener;
    robot_model_ = robot_model;

    int_marker_.name = name_;
    int_marker_.header.stamp=ros::Time(0);
    int_marker_.scale = 0.3;

    ros::NodeHandle n;
    targetPosePub_ = n.advertise<geometry_msgs::PoseStamped>(name_ + "/target_pose", 100);
    setRefContactClient_ = n.serviceClient<std_srvs::SetBool>(name_+"/set_ref_contact");
    setNearContactClient_ = n.serviceClient<std_srvs::SetBool>(name_+"/set_near_contact");
    setCaredClient_ = n.serviceClient<std_srvs::SetBool>(name_+"/set_cared");

    isEnabled_ = false;
  }
  void updateMarker(){
    if(!info_) return;
    int_marker_.description = name_ + ": " + state_;
    if(!isEnabled_){
      interactiveMarkerServer_->erase(name_);
      interactiveMarkerServer_->applyChanges();
    }else if(state_ == "NOT_CARED"){
      interactiveMarkerServer_->erase(name_);
      interactiveMarkerServer_->applyChanges();
    }else if(state_ == "AIR"){
      int_marker_.header.frame_id = "odom";
      int_marker_.pose = marker_pose_;

      // geometry_msgs::PoseStamped msg;
      // msg.header.frame_id = "odom";
      // msg.header.stamp = ros::Time::now();
      // msg.pose = current_pose_;
      // targetPosePub_.publish(msg);

      int_marker_.controls.clear();

      this->addMarkerControl();
      menu_handler_ = interactive_markers::MenuHandler();
      menu_handler_.insert( name_ +": Near Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->nearContactFeedBack(true);});
      menu_handler_.insert( name_ +": Not Cared", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->caredFeedBack(false);});

      this->add6DofControl();

      interactiveMarkerServer_->insert(int_marker_, [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->processFeedback(feedback);});
      menu_handler_.apply( *interactiveMarkerServer_, int_marker_.name );
    }else if(state_ == "NEAR_CONTACT"){
      int_marker_.header.frame_id = "odom";
      int_marker_.pose = marker_pose_;

      // geometry_msgs::PoseStamped msg;
      // msg.header.frame_id = "odom";
      // msg.header.stamp = ros::Time::now();
      // msg.pose = current_pose_;
      // targetPosePub_.publish(msg);

      int_marker_.controls.clear();

      this->addMarkerControl();
      menu_handler_ = interactive_markers::MenuHandler();
      menu_handler_.insert(name_ +": Ref Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->refContactFeedBack(true);});
      menu_handler_.insert( name_ +": Far Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->nearContactFeedBack(false);});

      this->add6DofControl();

      interactiveMarkerServer_->insert(int_marker_, [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->processFeedback(feedback);});
      menu_handler_.apply( *interactiveMarkerServer_, int_marker_.name );
    }else if(state_ == "TOWARD_MAKE_CONTACT"){
      int_marker_.header.frame_id = name_;
      int_marker_.pose = geometry_msgs::Pose();

      int_marker_.controls.clear();

      this->addMarkerControl(1);
      menu_handler_ = interactive_markers::MenuHandler();
      menu_handler_.insert( name_ +": Ref Not Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->refContactFeedBack(false);} );

      interactiveMarkerServer_->insert(int_marker_, [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->processFeedback(feedback);});
      menu_handler_.apply( *interactiveMarkerServer_, int_marker_.name );
    }else if(state_ == "CONTACT"){
      int_marker_.header.frame_id = name_;
      int_marker_.pose = geometry_msgs::Pose();

      int_marker_.controls.clear();

      this->addMarkerControl(2);
      menu_handler_ = interactive_markers::MenuHandler();
      menu_handler_.insert( name_ +": Ref Not Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->refContactFeedBack(false);} );

      interactiveMarkerServer_->insert(int_marker_, [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->processFeedback(feedback);});
      menu_handler_.apply( *interactiveMarkerServer_, int_marker_.name );
    }else if(state_ == "TOWARD_BREAK_CONTACT"){
      int_marker_.header.frame_id = name_;
      int_marker_.pose = geometry_msgs::Pose();

      int_marker_.controls.clear();

      this->addMarkerControl(1);
      menu_handler_ = interactive_markers::MenuHandler();
      menu_handler_.insert( name_ +": Ref Contact", [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->refContactFeedBack(true);} );

      interactiveMarkerServer_->insert(int_marker_, [&](const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){this->processFeedback(feedback);});
      menu_handler_.apply( *interactiveMarkerServer_, int_marker_.name );
    }
  }
  void refContactFeedBack(bool data){
    std_srvs::SetBool srv;
    srv.request.data = data;
    if(!setRefContactClient_.waitForExistence(ros::Duration(5))) return;
    setRefContactClient_.call(srv);
  }
  void nearContactFeedBack(bool data){
    std_srvs::SetBool srv;
    srv.request.data = data;
    if(!setNearContactClient_.waitForExistence(ros::Duration(5))) return;
    setNearContactClient_.call(srv);
  }
  void caredFeedBack(bool data){
    std_srvs::SetBool srv;
    srv.request.data = data;
    if(!setCaredClient_.waitForExistence(ros::Duration(5))) return;
    setCaredClient_.call(srv);
  }
  void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback){
    switch ( feedback->event_type ) {
      case visualization_msgs::InteractiveMarkerFeedback::POSE_UPDATE:
        if(state_ == "AIR" || state_ == "NEAR_CONTACT"){
          marker_pose_ = feedback->pose;
          geometry_msgs::PoseStamped msg;
          msg.header.frame_id = "odom";
          msg.header.stamp = ros::Time::now();
          msg.pose = marker_pose_;
          targetPosePub_.publish(msg);
        }
        break;
      default:
        break;
      }
    return;
  }
  void onInfoUpdated() override{
    this->updateMarker();
    interactiveMarkerServer_->applyChanges();}
  void onStateUpdated() override{
    if(state_ != prev_state_) {
      if((state_ == "AIR" || state_ == "NEAR_CONTACT") &&
         (prev_state_ != "AIR" && prev_state_ != "NEAR_CONTACT")){
        this->resetMarkerPose();
      }
      this->updateMarker();
      interactiveMarkerServer_->applyChanges();
    }
  }
  void addMarkerControl(int highlight=0){
    visualization_msgs::Marker marker;
    std::shared_ptr<const urdf::Link> link = robot_model_->getLink(info_->header.frame_id);
    if(link){
      // TODO local poseが考慮されなかったり、frame_idがメッシュのあるリンクを指していない場合エラーになったりするはず
      marker.type = visualization_msgs::Marker::MESH_RESOURCE;
      std::shared_ptr<const urdf::Mesh> mesh = std::dynamic_pointer_cast<const urdf::Mesh >(link->visual->geometry);
      marker.mesh_resource = mesh->filename;
    }
    Eigen::Affine3d trans;
    tf::transformMsgToEigen(info_->transform,trans);
    Eigen::Affine3d trans_inv = trans.inverse();
    tf::poseEigenToMsg(trans_inv,marker.pose);
    marker.scale.x = marker.scale.y = marker.scale.z = 1.0;
    marker.mesh_use_embedded_materials = true;
    switch(highlight){
    case 0:
      marker.color.a = 0.5;
      break;
    case 1:
      marker.color.b = 1.0;
      marker.color.a = 1.0;
      break;
    case 2:
      marker.color.r = 1.0;
      marker.color.a = 1.0;
      break;
    default:
      break;
    }
    visualization_msgs::InteractiveMarkerControl menu_control;
    menu_control.always_visible = true;
    menu_control.name = name_ + "_menu";
    menu_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MENU;
    menu_control.markers.push_back( marker );
    int_marker_.controls.push_back( menu_control );
  }
  void add6DofControl(){
    visualization_msgs::InteractiveMarkerControl control;
    control.orientation.w = 1;
    control.orientation.x = 1;
    control.orientation.y = 0;
    control.orientation.z = 0;
    control.name = "rotate_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_x";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 1;
    control.orientation.z = 0;
    control.name = "rotate_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_z";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);

    control.orientation.w = 1;
    control.orientation.x = 0;
    control.orientation.y = 0;
    control.orientation.z = 1;
    control.name = "rotate_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
    int_marker_.controls.push_back(control);
    control.name = "move_y";
    control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
    int_marker_.controls.push_back(control);
    return;
  }
  void resetMarkerPose(){
    if (!tfListener_->waitForTransform("odom", name_, ros::Time(0), ros::Duration(1.0))) {
      ROS_ERROR("[EndEffectorInteractiveMarkerServer::updateMarker] failed to lookup transform between %s and %s", "odom", name_.c_str());
    }else{
      tf::StampedTransform transform;
      tfListener_->lookupTransform("odom", name_, ros::Time(0), transform);
      Eigen::Affine3d eigenpose;
      tf::transformTFToEigen(transform,eigenpose);
      tf::poseEigenToMsg(eigenpose,marker_pose_);
    }
  }

  bool isEnabled() const {return isEnabled_;}
  bool& isEnabled() {return isEnabled_;}
private:
  bool isEnabled_;

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer_;
  std::shared_ptr<tf::TransformListener> tfListener_;
  std::shared_ptr<urdf::Model> robot_model_;
  ros::Publisher targetPosePub_;
  ros::ServiceClient setRefContactClient_;
  ros::ServiceClient setNearContactClient_;
  ros::ServiceClient setCaredClient_;

  visualization_msgs::InteractiveMarker int_marker_;
  interactive_markers::MenuHandler menu_handler_;
  geometry_msgs::Pose marker_pose_;
};

int main(int argc, char** argv){
  ros::init(argc,argv,"end_effector_interactive_marker_server");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::shared_ptr<interactive_markers::InteractiveMarkerServer> interactiveMarkerServer = std::make_shared<interactive_markers::InteractiveMarkerServer>("end_effector_interactive_marker_server");
  std::shared_ptr<tf::TransformListener> tfListener = std::make_shared<tf::TransformListener>();
  std::shared_ptr<urdf::Model> robot_model = std::make_shared<urdf::Model>();
  robot_model->initParam("robot_description");

  bool isEnabled;
  pnh.param("start_enabled",isEnabled,false);

  std::map<std::string,std::shared_ptr<EndEffectorInteractiveMarkerController> > endEffectors;
  ros::Subscriber endEffectorsSub
    = nh.subscribe<multicontact_controller_msgs::StringArray>
    ("end_effectors", 10,
     [&](const multicontact_controller_msgs::StringArray::ConstPtr& msg){
      multicontact_controller::endeffectorutils::stringArrayToEndEffectors(msg, endEffectors ,interactiveMarkerServer,tfListener,robot_model);
      for(std::map<std::string,std::shared_ptr<EndEffectorInteractiveMarkerController> >::iterator it = endEffectors.begin(); it != endEffectors.end(); it++) {
        it->second->isEnabled() = isEnabled;
      }
     });

  ros::ServiceServer enableService
    = pnh.advertiseService<std_srvs::SetBool::Request, std_srvs::SetBool::Response>
    ("enable",
     [&](std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
      if(isEnabled == request.data){
        ROS_INFO("[MultiContactFootCoords::enableService] Already %s",(isEnabled ? "Enabled" : "Disabled"));
        response.success = true;
        response.message = "";
        return true;
      } else {
        isEnabled = request.data;
        ROS_INFO("[MultiContactFootCoords::enableService] %s",(isEnabled ? "Enabled" : "Disabled"));

        for(std::map<std::string,std::shared_ptr<EndEffectorInteractiveMarkerController> >::iterator it = endEffectors.begin(); it != endEffectors.end(); it++) {
          it->second->isEnabled() = isEnabled;
          if(isEnabled){
            if(it->second->state() == "AIR" || it->second->state() == "NEAR_CONTACT") it->second->resetMarkerPose();
          }
          it->second->updateMarker();
        }
        interactiveMarkerServer->applyChanges();

        response.success = true;
        response.message = "";
        return true;
      }});

  int rate;
  pnh.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  unsigned int seq = 0;
  while (ros::ok()) {
    // spin
    ros::spinOnce();

    ros::Time now = ros::Time::now();


    seq++;
    r.sleep();
  }

  return 0;
}
