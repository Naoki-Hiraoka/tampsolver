#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/BodyLoader>
#include <multicontact_controller_msgs/CollisionArray.h>

namespace multicontact_controller {
  std::vector<cnoid::Link*> EndEffectorPCLCDROS::allowCollisionLinks(){
    if(this->state_ == "CONTACT" ||
       this->state_ == "TOWARD_MAKE_CONTACT" ||
       this->state_ == "TOWARD_BREAK_CONTACT" ||
       this->state_ == "NEAR_CONTACT"){
      return allowCollisionLinks_;
    }else{
      return std::vector<cnoid::Link*>();
    }
  }

  void EndEffectorPCLCDROS::onInfoUpdated(){
    endeffectorutils::updateContactPointFromInfo(robot_, contactPoint_, *info_);

    allowCollisionLinks_.clear();
    for(size_t i=0;i<info_->collision.allow_collision_links.size();i++){
      cnoid::Link* link = robot_->link(info_->collision.allow_collision_links[i]);
      if(!link){
        ROS_WARN("%s not found",info_->collision.allow_collision_links[i].c_str());
        continue;
      }
      allowCollisionLinks_.push_back(link);
    }

    {
      const multicontact_controller_msgs::BoundingBox& boundingBox = info_->collision.allow_collision_box;
      if(boundingBox.size.x >= 0.0 &&
         boundingBox.size.y >= 0.0 &&
         boundingBox.size.z >= 0.0){
        allowCollisionBoxFilter_->setMax(Eigen::Vector4f(boundingBox.size.x/2,
                                                         boundingBox.size.y/2,
                                                         boundingBox.size.z/2,
                                                         1.0));
        allowCollisionBoxFilter_->setMin(Eigen::Vector4f(-boundingBox.size.x/2,
                                                         -boundingBox.size.y/2,
                                                         -boundingBox.size.z/2,
                                                         1.0));
      }else{
        ROS_ERROR("boundingBox.size < 0");
      }

      {
        cnoid::Vector3 p;
        tf::vectorMsgToEigen(boundingBox.center, p);
        allowCollisionBoxCenter_.translation() = p;
      }
    }
  }

  std::vector<cnoid::SgNodePtr> EndEffectorPCLCDROS::getDrawOnObjects(){
    if(!this->isValid() ||
       !(this->state_ == "CONTACT" ||
         this->state_ == "TOWARD_MAKE_CONTACT" ||
         this->state_ == "TOWARD_BREAK_CONTACT" ||
         this->state_ == "NEAR_CONTACT")){
      return std::vector<cnoid::SgNodePtr>();
    }

    if(!this->lines){
      this->lines = new cnoid::SgLineSet;
      this->lines->setLineWidth(3.0);
      this->lines->getOrCreateColors()->resize(1);
      this->lines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,1.0,0.0);
      // A, B
      this->lines->getOrCreateVertices()->resize(8);
      this->lines->colorIndices().resize(0);
      this->lines->addLine(0,1); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(1,2); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(2,3); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(3,0); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(0,4); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(1,5); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(2,6); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(3,7); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(4,5); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(5,6); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(6,7); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      this->lines->addLine(7,4); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
    }

    cnoid::Vector4f maxVec = allowCollisionBoxFilter_->getMax();
    cnoid::Vector4f minVec = allowCollisionBoxFilter_->getMin();
    Eigen::Affine3f center = (this->contactPoint_->parent()->T() * this->contactPoint_->T_local() * this->allowCollisionBoxCenter_).cast<float>();

    this->lines->vertices()->at(0) = center * cnoid::Vector3f(minVec[0],minVec[1],minVec[2]);
    this->lines->vertices()->at(1) = center * cnoid::Vector3f(maxVec[0],minVec[1],minVec[2]);
    this->lines->vertices()->at(2) = center * cnoid::Vector3f(maxVec[0],maxVec[1],minVec[2]);
    this->lines->vertices()->at(3) = center * cnoid::Vector3f(minVec[0],maxVec[1],minVec[2]);
    this->lines->vertices()->at(4) = center * cnoid::Vector3f(minVec[0],minVec[1],maxVec[2]);
    this->lines->vertices()->at(5) = center * cnoid::Vector3f(maxVec[0],minVec[1],maxVec[2]);
    this->lines->vertices()->at(6) = center * cnoid::Vector3f(maxVec[0],maxVec[1],maxVec[2]);
    this->lines->vertices()->at(7) = center * cnoid::Vector3f(minVec[0],maxVec[1],maxVec[2]);

    return std::vector<cnoid::SgNodePtr>{this->lines};
  }

  void PCLCollisionDetectorROS::main(int argc, char** argv) {
    ros::init(argc,argv,"pcl_collision_detector");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    // initialize robot
    // 質量パラメータのキャリブ TODO
    robot_ = cnoidbodyutils::loadBodyFromParam("/vrml_file");
    if(!robot_){
      ROS_ERROR("Failed to load robot from '/vrml_file'");
      return;
    }
    objects(robot_);

    pclCollisionDetector_ = std::make_shared<PCLCollisionDetector>(robot_);
    {
      double boxelSize;
      pnh.param("boxel_size", boxelSize, 0.02);
      pclCollisionDetector_->setBoxelSize(boxelSize);
    }

    // setup subscribers
    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &PCLCollisionDetectorROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber odomSub = nh.subscribe("odom", 1, &PCLCollisionDetectorROS::odomCallback, this);

    ros::Subscriber obstacleSub = nh.subscribe("obstacle_model", 1, &PCLCollisionDetectorROS::obstacleCallback, this);

    ros::Subscriber endEffectorsSub = nh.subscribe("end_effectors", 1, &PCLCollisionDetectorROS::endEffectorsCallback, this);

    // setup publishers
    ros::Publisher collisionArrayPub =  nh.advertise<multicontact_controller_msgs::CollisionArray>("pcl_collision", 100);

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


    pnh.param("start_enabled", isEnabled_, true);
    ros::ServiceServer enableService = pnh.advertiseService("enable",&PCLCollisionDetectorROS::enableCallback,this);

    // main loop
    int rate;
    pnh.param("rate", rate, 100); // 100 hz
    ros::Rate r(rate);

    multicontact_controller_msgs::CollisionArray msg;

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
        std::map<cnoid::Link*, std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > > > allowCollisionBoxFilters;
        getAllowCollisionBoxFilters(endEffectors_,allowCollisionBoxFilters);
        pclCollisionDetector_->setLinks(environmentCollisionLinks);
        pclCollisionDetector_->setAllowCollisionBoxFilters(allowCollisionBoxFilters);
        if(pclCollisionDetector_->solve()){
          msg.header.seq = seq;
          msg.header.stamp = now;

          std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs = pclCollisionDetector_->collisionLinkPairs();
          msg.collisions.resize(collisionLinkPairs.size());
          for(size_t i=0;i<collisionLinkPairs.size();i++){
            const std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = collisionLinkPairs[i];
            cnoidbodyutils::collisionLinkPairToMsg(collisionLinkPairs[i],msg.collisions[i]);
          }

          collisionArrayPub.publish(msg);
        }else{
          ROS_WARN("pclCollisionDetector failed");
        }
      }

      if(this->hasViewer()){
        this->drawObjects(false);
        if(isEnabled_){
          {
            std::vector<cnoid::SgNodePtr> objects = pclCollisionDetector_->getDrawOnObjects();
            for(size_t j=0;j<objects.size();j++){
              this->drawOn(objects[j]);
            }
          }
          for(std::map<std::string,std::shared_ptr<EndEffectorPCLCDROS> >::iterator it=endEffectors_.begin();it!=endEffectors_.end();it++){
            std::vector<cnoid::SgNodePtr> objects = it->second->getDrawOnObjects();
            for(size_t j=0;j<objects.size();j++){
              this->drawOn(objects[j]);
            }
          }
        }
        this->flush();
      }

      seq++;
      stamp = now;

      drawObjects();

      r.sleep();
    }

    exit(0);

  }

  void PCLCollisionDetectorROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 加速度がない TODO
    if(robot_){
      cnoidbodyutils::jointStateToBody(msg,robot_);
    }
  }

  void PCLCollisionDetectorROS::odomCallback(const nav_msgs::Odometry::ConstPtr& msg){
    if(robot_){
      cnoidbodyutils::odomToBody(msg, robot_);
    }
  }

  void PCLCollisionDetectorROS::obstacleCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){
    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *obstacle_model);
    if(obstacle_model->points.size() > 0){
      pclCollisionDetector_->setObstacleModel(obstacle_model);
    }
  }

  void PCLCollisionDetectorROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
    endeffectorutils::stringArrayToEndEffectors(msg,endEffectors_,this->robot_);
  }

  bool PCLCollisionDetectorROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
