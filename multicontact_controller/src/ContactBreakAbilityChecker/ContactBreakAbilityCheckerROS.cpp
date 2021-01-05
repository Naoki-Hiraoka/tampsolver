#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROS.h>

#include <eigen_conversions/eigen_msg.h>
#include <tf2_ros/transform_broadcaster.h>
#include <jsk_rviz_plugins/OverlayText.h>

namespace multicontact_controller {
  void EndEffectorCBACROS::onInfoUpdated(){
    EndEffectorPCLCDROS::onInfoUpdated();
    cnoidbodyutils::loadContactFromInfo(info_->contact, std::dynamic_pointer_cast<ContactPointCBAC>(contactPoint_)->contact());
  }

  void EndEffectorCBACROS::onStateUpdated(){
    EndEffectorPCLCDROS::onStateUpdated();
    if(this->isValid()){
      std::dynamic_pointer_cast<ContactPointCBAC>(contactPoint_)->state() = state_;
    }
  }

  std::vector<cnoid::SgNodePtr> EndEffectorCBACROS::getDrawOnObjects(){
    std::vector<cnoid::SgNodePtr> drawOnObjects = EndEffectorPCLCDROS::getDrawOnObjects();
    if(this->isValid()){
      std::vector<cnoid::SgNodePtr> objects = this->contactPoint()->getDrawOnObjects();
      std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
    }
    return drawOnObjects;
  }

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

    cnoidbodyutils::setupJointInfosFromParam(robot_, jointInfos_, jointInfoMap_);

    contactBreakAbilityChecker_ = std::make_shared<ContactBreakAbilityChecker>(robot_, jointInfos_);
    if(this->hasViewer()){
      contactBreakAbilityChecker_->setIKLoopCb([this](){ this->draw(); });
    }

    selfCollisionDetector_ = std::make_shared<SelfCollisionDetector>(robot_);
    setupCollisionPairFromParam(robot_, selfCollisionDetector_->collisionLinkPairs());

    pclCollisionDetector_ = std::make_shared<PCLCollisionDetector>(robot_);
    {
      double boxelSize;
      pnh.param("boxel_size", boxelSize, 0.02);
      pclCollisionDetector_->setBoxelSize(boxelSize);
    }

    ros::Publisher overlayTextPub = pnh.advertise<jsk_rviz_plugins::OverlayText>("contact_breakability_text", 1);

    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &ContactBreakAbilityCheckerROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber odomSub = nh.subscribe("odom", 1, &ContactBreakAbilityCheckerROS::odomCallback, this);

    ros::Subscriber obstacleSub = nh.subscribe("obstacle_model", 1, &ContactBreakAbilityCheckerROS::obstacleCallback, this);

    ros::Subscriber endEffectorsSub = nh.subscribe("end_effectors", 1, &ContactBreakAbilityCheckerROS::endEffectorsCallback, this);

    std::vector<cnoid::Link*> environmentCollisionLinks;
    {
      std::vector<std::string> environmentCollisionLink_name;
      nh.param("environment_collision_link", environmentCollisionLink_name, std::vector<std::string>());
      for(size_t i=0;i<environmentCollisionLink_name.size();i++){
        cnoid::Link* link = robot_->link(environmentCollisionLink_name[i]);
        if(!link){
          ROS_ERROR("%s not found",environmentCollisionLink_name[i].c_str());
          continue;
        }
        environmentCollisionLinks.push_back(link);
      }
    }

    pnh.param("start_enabled", isEnabled_, false);
    ros::ServiceServer enableService = pnh.advertiseService("enable",&ContactBreakAbilityCheckerROS::enableCallback,this);

    // main loop
    int rate;
    pnh.param("rate", rate, 1); // 1 hz
    ros::Rate r(rate);

    jsk_rviz_plugins::OverlayText msg;
    msg.width = 400;
    msg.height = 20;
    msg.left = 10;
    msg.top = 10;
    msg.text_size = 12;
    msg.line_width = 2;
    msg.font = "DejaVu Sans Mono";
    msg.text = "Stability Margins [m]\n";
    msg.fg_color.r =25 / 255.0;
    msg.fg_color.g = 1.0;
    msg.fg_color.b = 240.0 / 255.0;
    msg.fg_color.a = 1.0;
    msg.bg_color.r = 0.0;
    msg.bg_color.g = 0.0;
    msg.bg_color.b = 0.0;
    msg.bg_color.a = 0.2;

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();
      msg.text = "Stability Margins [m]\n";
      msg.height = 20;

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
        std::map<cnoid::Link*, std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > > > allowCollisionBoxFilters;
        getAllowCollisionBoxFilters(endEffectors_,allowCollisionBoxFilters);

        pclCollisionDetector_->setLinks(environmentCollisionLinks);
        pclCollisionDetector_->setAllowCollisionBoxFilters(allowCollisionBoxFilters);
        pclCollisionDetector_->filterDistanceGlobal() = 1.0;
        pclCollisionDetector_->filterDistanceLocal() = 0.45;

        std::vector<std::shared_ptr<ContactPointCBAC> > contactPoints;
        for(std::map<std::string,std::shared_ptr<EndEffectorCBACROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
          if(it->second->state() != "NOT_CARED" &&
             it->second->isValid()){
            contactPoints.push_back(it->second->contactPoint());
          }
        }

        std::vector<double> initialAngleVector(robot_->numJoints());
        for(size_t i=0;i<robot_->numJoints();i++) initialAngleVector[i] = robot_->joint(i)->q();
        cnoid::Position initialRootPosition = robot_->rootLink()->T();

        for(size_t i=0;i<contactPoints.size();i++){
          if(contactPoints[i]->state() != "CONTACT" &&
             contactPoints[i]->state() != "TOWARD_BREAK_CONTACT") continue;

          double margin;
          contactBreakAbilityChecker_->check(margin,
                                             contactPoints,
                                             contactPoints[i],
                                             true,
                                             selfCollisionDetector_,
                                             pclCollisionDetector_);

          if(margin > 0){
            msg.text += "Without " + contactPoints[i]->name() + " Contact: <span style=\"color: green;\">" + std::to_string(margin) + "</span>\n";
            msg.height += msg.text_size * 2.0;
          }else{
            msg.text += "Without " + contactPoints[i]->name() + " Contact: <span style=\"color: red;\">" + std::to_string(margin) + "</span>\n";
            msg.height += msg.text_size * 2.0;
          }

          if(this->hasViewer()){
            cnoidbodyutils::copyBodyKinematicsState(robot_,endEffectors_[contactPoints[i]->name()]->drawRobot());
          }
          for(size_t j=0;j<robot_->numJoints();j++) robot_->joint(j)->q() = initialAngleVector[j];
          robot_->rootLink()->T() = initialRootPosition;
          robot_->calcForwardKinematics(false,false);
        }
      }

      overlayTextPub.publish(msg);

      if(this->hasViewer()){
        this->draw();
      }
      seq++;
      stamp = now;

      r.sleep();
    }

    exit(0);

    }

  void ContactBreakAbilityCheckerROS::draw(){

    if(isEnabled_){
      std::vector<cnoid::Body*> drawRobots{robot_};
      for(std::map<std::string,std::shared_ptr<EndEffectorCBACROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
        if(it->second->isValid() &&
           (it->second->state() == "CONTACT" || it->second->state() == "TOWARD_BREAK_CONTACT")){
          drawRobots.push_back(it->second->drawRobot());
        }
      }

      this->objects(drawRobots);

      this->drawObjects(false);

      {
        std::vector<cnoid::SgNodePtr> objects = contactBreakAbilityChecker_->getDrawOnObjects();
        for(size_t j=0;j<objects.size();j++){
          this->drawOn(objects[j]);
        }
      }

      {
        std::vector<cnoid::SgNodePtr> objects = selfCollisionDetector_->getDrawOnObjects();
        for(size_t j=0;j<objects.size();j++){
          this->drawOn(objects[j]);
        }
      }

      {
        std::vector<cnoid::SgNodePtr> objects = pclCollisionDetector_->getDrawOnObjects();
        for(size_t j=0;j<objects.size();j++){
          this->drawOn(objects[j]);
        }
      }

      for(std::map<std::string,std::shared_ptr<EndEffectorCBACROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
        std::vector<cnoid::SgNodePtr> objects = it->second->getDrawOnObjects();
        for(size_t j=0;j<objects.size();j++){
          this->drawOn(objects[j]);
        }
      }
    }else{
      std::vector<cnoid::Body*> drawRobots{robot_};
      this->objects(drawRobots);
      this->drawObjects(false);
    }

    this->flush();

  }

  void ContactBreakAbilityCheckerROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if(robot_){
      cnoidbodyutils::jointStateToBody(msg,robot_);
    }
  }

  void ContactBreakAbilityCheckerROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if(robot_){
      cnoidbodyutils::odomToBody(msg, robot_);
    }
  }

  void ContactBreakAbilityCheckerROS::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *obstacle_model);
    pclCollisionDetector_->setObstacleModel(obstacle_model);
  }

  void ContactBreakAbilityCheckerROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
    endeffectorutils::stringArrayToEndEffectors(msg,endEffectors_,this->robot_);
  }

  bool ContactBreakAbilityCheckerROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
