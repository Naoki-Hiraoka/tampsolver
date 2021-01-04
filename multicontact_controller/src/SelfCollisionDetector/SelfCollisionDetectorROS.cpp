#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetectorROS.h>

#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/BodyLoader>
#include <multicontact_controller_msgs/CollisionArray.h>

namespace multicontact_controller {
  void setupCollisionPairFromParam(cnoid::Body* robot, std::vector<std::shared_ptr<cnoid::CollisionLinkPair> >& collisionLinkPairs){
    ros::NodeHandle nh;

    std::vector<std::string> collision_pair;
    nh.param("collision_pair", collision_pair, std::vector<std::string>());
    for(size_t i=0;i<collision_pair.size();i++){
      std::stringstream ss(collision_pair[i]);
      std::string link0_name, link1_name;
      std::getline(ss,link0_name,':');
      std::getline(ss,link1_name);

      std::shared_ptr<cnoid::CollisionLinkPair> collisionLinkPair = std::make_shared<cnoid::CollisionLinkPair>();
      collisionLinkPair->body[0] = robot;
      collisionLinkPair->body[1] = robot;
      collisionLinkPair->link[0] = robot->link(link0_name);
      collisionLinkPair->link[1] = robot->link(link1_name);
      if(!collisionLinkPair->link[0]){
        ROS_ERROR("%s not found",link0_name.c_str());
        continue;
      }
      if(!collisionLinkPair->link[1]){
        ROS_ERROR("%s not found",link1_name.c_str());
        continue;
      }
      collisionLinkPairs.push_back(collisionLinkPair);
    }
  }

  void SelfCollisionDetectorROS::main(int argc, char** argv) {

    ros::init(argc,argv,"self_collision_detector");
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

    // setup subscribers
    ros::Subscriber jointStateSub = nh.subscribe("joint_states", 100, &SelfCollisionDetectorROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    // setup publishers
    ros::Publisher collisionArrayPub =  nh.advertise<multicontact_controller_msgs::CollisionArray>("self_collision", 100);

    selfCollisionDetector_ = std::make_shared<SelfCollisionDetector>(robot_);
    setupCollisionPairFromParam(robot_, selfCollisionDetector_->collisionLinkPairs());

    pnh.param("start_enabled", isEnabled_, true);
    ros::ServiceServer enableService = pnh.advertiseService("enable",&SelfCollisionDetectorROS::enableCallback,this);

    // main loop
    int rate;
    pnh.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    multicontact_controller_msgs::CollisionArray msg;
    for(size_t i=0;i<selfCollisionDetector_->collisionLinkPairs().size();i++){
      multicontact_controller_msgs::Collision collision;
      collision.point1.header.frame_id = selfCollisionDetector_->collisionLinkPairs()[i]->link[0]->name();
      collision.point2.header.frame_id = selfCollisionDetector_->collisionLinkPairs()[i]->link[1]->name();
      collision.normal1.header.frame_id = selfCollisionDetector_->collisionLinkPairs()[i]->link[0]->name();
      collision.normal2.header.frame_id = selfCollisionDetector_->collisionLinkPairs()[i]->link[1]->name();
      msg.collisions.push_back(collision);
    }

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
        if(selfCollisionDetector_->solve()){
          msg.header.seq = seq;
          msg.header.stamp = now;
          for(size_t i=0;i<selfCollisionDetector_->collisionLinkPairs().size();i++){
            const std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = selfCollisionDetector_->collisionLinkPairs()[i];
            multicontact_controller_msgs::Collision& collision = msg.collisions[i];

            collision.point1.header.stamp = now;
            collision.point1.header.seq = seq;
            tf::pointEigenToMsg(collisionLinkPair->collisions[0].point,collision.point1.point);
            tf::vectorEigenToMsg(collisionLinkPair->collisions[0].normal,collision.normal1.vector);
            collision.point2.header.stamp = now;
            collision.point2.header.seq = seq;
            tf::pointEigenToMsg(collisionLinkPair->collisions[1].point,collision.point2.point);
            tf::vectorEigenToMsg(collisionLinkPair->collisions[1].normal,collision.normal2.vector);
            collision.distance = - collisionLinkPair->collisions[0].depth;
          }

          collisionArrayPub.publish(msg);
        }else{
          ROS_WARN("selfCollisionDetector failed");
        }
      }

      if(this->hasViewer()){
        this->drawObjects(false);
        if(isEnabled_){
          std::vector<cnoid::SgNodePtr> objects = selfCollisionDetector_->getDrawOnObjects();
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

  void SelfCollisionDetectorROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 加速度がない TODO
    if(robot_){
      cnoidbodyutils::jointStateToBody(msg,robot_);
    }
  }

  bool SelfCollisionDetectorROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
