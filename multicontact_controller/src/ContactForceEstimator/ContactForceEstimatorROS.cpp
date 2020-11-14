#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROS.h>

#include <ros/ros.h>
#include <eigen_conversions/eigen_msg.h>
#include <cnoid/ForceSensor>
#include <cnoid/BodyLoader>
#include <tf2_ros/buffer_client.h>

namespace multicontact_controller {
  void ContactForceEstimatorROS::main(int argc, char** argv) {
    ros::init(argc,argv,"ContactForceEstimator");
    ros::NodeHandle n;

    // initialize robot
    // 質量パラメータのキャリブ TODO
    std::string vrml_file;
    if (!n.getParam("vrml_file", vrml_file)) {
      ROS_ERROR("Failed to get param 'vrml_file'");
      return;
    }
    cnoid::BodyLoader loader;
    robot_ = loader.load(vrml_file);
    if(!robot_){
      ROS_ERROR("Failed to load %s", vrml_file.c_str());
      return;
    }

    std::string robot_description;
    if (!n.getParam("/robot_description", robot_description)) {
      ROS_ERROR("Failed to get param '/robot_description'");
      return;
    }
    model_.initString(robot_description);



    // setup subscribers
    ros::Subscriber jointStateSub = n.subscribe("joint_states", 1, &ContactForceEstimatorROS::jointStateCallback, this);

    ros::Subscriber imuSub = n.subscribe("imu", 1, &ContactForceEstimatorROS::imuCallback, this);

    std::vector<ros::Subscriber> forceSensorSub;
    if(robot_){
      cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
      for(size_t i=0;i<forceSensors.size();i++){
        forceSensorSub.push_back(n.subscribe(forceSensors[i]->name(), 1, &ContactForceEstimatorROS::forceSensorCallback, this));
      }
    }

    ros::Subscriber contactPointsSub = n.subscribe("contactPoints", 1, &ContactForceEstimatorROS::contactPointsCallback, this);

    std::string tf2_buffer_server_ns;
    n.param("tf2_buffer_server_ns", tf2_buffer_server_ns, std::string("tf2_buffer_server"));
    tf2_ros::BufferClient tf2client(tf2_buffer_server_ns,10000);

    // setup publishers
    std::map<std::string,ros::Publisher> contactForcePub;


    // setup ContactForceEstimator
    ContactForceEstimator contactForceEstimator;
    contactForceEstimator.setRobot(robot_);


    // main loop
    ros::Rate r(250); // 250 hz
    unsigned int seq = 0;
    while (ros::ok()) {
      // spin
      ros::spinOnce();

      // update candidatepoints
      contactForceEstimator.clearCandidatePoints();
      for(size_t i=0;i<contactPoints_.size();i++){
        contactForceEstimator.setCandidatePoint(contactPoints_[i]);
      }

      for(size_t i=0;i<contactPoints_.size();i++){
        geometry_msgs::TransformStamped transformStamped;
        try{
          transformStamped = tf2client.lookupTransform(contactPoints_[i]->parent()->name(),contactPoints_[i]->name(),ros::Time(0));
        } catch (tf2::LookupException ex) {
          ROS_WARN("%s",ex.what());
          continue;
        } catch (tf2::ConnectivityException ex) {
          ROS_WARN("%s",ex.what());
          continue;
        } catch (tf2::ExtrapolationException ex) {
          ROS_WARN("%s",ex.what());
          continue;
        } catch (tf2::InvalidArgumentException ex){
          ROS_WARN("%s",ex.what());
          continue;
        }

        cnoid::Vector3 p;
        tf::vectorMsgToEigen(transformStamped.transform.translation,p);
        contactPoints_[i]->T_local().translation() = p;
        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen(transformStamped.transform.rotation,q);
        contactPoints_[i]->T_local().linear() = q.normalized().toRotationMatrix();
      }

      //estimate
      contactForceEstimator.estimateForce();

      //publish
      ros::Time now = ros::Time::now();
      for(size_t i=0;i<contactPoints_.size();i++){
        cnoid::Vector6 F = contactForceEstimator.getEstimatedForce(contactPoints_[i]->name());

        if(contactForcePub.find(contactPoints_[i]->name()) == contactForcePub.end()){
          contactForcePub[contactPoints_[i]->name()] = n.advertise<geometry_msgs::WrenchStamped>(contactPoints_[i]->name()+"force", 1000);
        }

        geometry_msgs::WrenchStamped msg;
        msg.header.seq = seq;
        msg.header.stamp = now;
        msg.header.frame_id = contactPoints_[i]->name();
        tf::wrenchEigenToMsg(F, msg.wrench);
        contactForcePub[contactPoints_[i]->name()].publish(msg);
      }
      seq++;

      r.sleep();
    }

  }

  void ContactForceEstimatorROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    // 加速度がない TODO
    if(robot_){
      for(size_t i=0;i<msg->name.size();i++){
        cnoid::Link* joint = robot_->link(msg->name[i]);
        if(!joint) continue;
        if(msg->position.size() == msg->name.size()) joint->q() = msg->position[i];
        if(msg->velocity.size() == msg->name.size()) joint->dq() = msg->velocity[i];
        if(msg->effort.size() == msg->name.size()) joint->u() = msg->effort[i];
      }

      robot_->calcForwardKinematics(false,false);
    }
  }

  void ContactForceEstimatorROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){
    if(robot_){
      // rootのvel, accがない TODO
      cnoid::Device* device = robot_->findDevice(msg->header.frame_id);
      if(!device) return;
      cnoid::Position T = device->link()->T() * device->T_local();
      cnoid::Position realT;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(msg->orientation,q);
      realT.linear() = q.normalized().toRotationMatrix();
      robot_->rootLink()->T() = realT * T.inverse() * robot_->rootLink()->T();
      robot_->calcForwardKinematics(false,false);
    }
  }

  void ContactForceEstimatorROS::forceSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg) {
    if(robot_){
      cnoid::ForceSensor* device = robot_->findDevice<cnoid::ForceSensor>(msg->header.frame_id);
      if(!device) return;
      tf::wrenchMsgToEigen(msg->wrench,device->F());
    }
  }

  void ContactForceEstimatorROS::contactPointsCallback(const multicontact_controller_msgs::ContactPoints::ConstPtr& msg) {
    std::vector<std::shared_ptr<ContactPoint> > nextContactPoints;
    for(size_t i=0;i<msg->contactpoints.size();i++){
      std::vector<std::shared_ptr<ContactPoint> >::iterator result = std::find_if(contactPoints_.begin(),contactPoints_.end(),[&](std::shared_ptr<ContactPoint> p){return p->name()==msg->contactpoints[i].header.frame_id;});
      if( result != contactPoints_.end() ){
        //choreonoidのロボットモデルはリンク名が関節名によって管理されている
        urdf::LinkConstSharedPtr link = model_.getLink(msg->contactpoints[i].link);
        if(!link)continue;
        urdf::JointSharedPtr joint = link->parent_joint;
        cnoid::Link* parent;
        if(!joint) parent = robot_->link(link->name);
        else  parent = robot_->link(joint->name);
        if(!parent) continue;
        (*result)->parent() = parent;
        nextContactPoints.push_back(*result);
      } else {
        std::shared_ptr<ContactPoint> p = std::make_shared<ContactPoint>();
        p->name() = msg->contactpoints[i].header.frame_id;
        cnoid::Link* parent = robot_->link(msg->contactpoints[i].link);
        p->parent() = parent;
        p->T_local().setIdentity();
        nextContactPoints.push_back(p);
      }
    }
    contactPoints_ = nextContactPoints;
  }
};
