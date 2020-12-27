#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/BodyLoader>
#include <multicontact_controller_msgs/CollisionArray.h>

namespace multicontact_controller {
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

    ros::Subscriber odomSub = nh.subscribe("multicontact_odom", 1, &PCLCollisionDetectorROS::odomCallback, this);

    ros::Subscriber obstacleSub = nh.subscribe("/octomap/octomap_point_cloud_centers", 1, &PCLCollisionDetectorROS::obstacleCallback, this);

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
        std::vector<cnoid::Link*> links = environmentCollisionLinks;
        pclCollisionDetector_->setLinks(links);
        if(pclCollisionDetector_->solve()){
          msg.header.seq = seq;
          msg.header.stamp = now;

          std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs = pclCollisionDetector_->collisionLinkPairs();
          {
            size_t idx = 0;
            for(size_t i=0;i<collisionLinkPairs.size();i++){
              idx+=collisionLinkPairs[i]->collisions.size()/2;
            }
            msg.collisions.resize(idx);
          }
          size_t idx = 0;
          for(size_t i=0;i<collisionLinkPairs.size();i++){
            const std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = collisionLinkPairs[i];
            for(size_t j=0;j<collisionLinkPairs[i]->collisions.size()/2;j++){
              multicontact_controller_msgs::Collision& collision = msg.collisions[idx];
              collision.point1.header.stamp = now;
              collision.point1.header.seq = seq;
              collision.point1.header.frame_id = collisionLinkPair->link[0]->name();
              tf::pointEigenToMsg(collisionLinkPair->link[0]->T().inverse() * collisionLinkPair->collisions[j*2+0].point,collision.point1.point);
              collision.normal1.header.frame_id = collisionLinkPair->link[0]->name();
              tf::vectorEigenToMsg(collisionLinkPair->link[0]->T().inverse() * collisionLinkPair->collisions[j*2+0].normal,collision.normal1.vector);
              collision.point2.header.stamp = now;
              collision.point2.header.seq = seq;
              collision.point2.header.frame_id = "odom";
              tf::pointEigenToMsg(collisionLinkPair->collisions[j*2+1].point,collision.point2.point);
              collision.normal2.header.frame_id = "odom";
              tf::vectorEigenToMsg(collisionLinkPair->collisions[j*2+1].normal,collision.normal2.vector);

              collision.distance = - collisionLinkPair->collisions[j*2+0].depth;

              idx++;
            }
          }

          collisionArrayPub.publish(msg);
        }else{
          ROS_WARN("pclCollisionDetector failed");
        }
      }

      if(this->hasViewer()){
        this->drawObjects(false);
        if(isEnabled_){
          std::vector<cnoid::SgNodePtr> objects = pclCollisionDetector_->getDrawOnObjects();
          for(size_t j=0;j<objects.size();j++){
            this->drawOn(objects[j]);
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
    pclCollisionDetector_->setObstacleModel(obstacle_model);
  }

  bool PCLCollisionDetectorROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
