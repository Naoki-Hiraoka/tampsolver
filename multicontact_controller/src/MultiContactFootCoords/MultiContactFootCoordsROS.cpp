#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROS.h>

#include <eigen_conversions/eigen_msg.h>

namespace multicontact_controller {

  void MultiContactFootCoordsROS::main(int argc, char** argv) {

    ros::init(argc,argv,"contact_force_estimator");
    ros::NodeHandle n;
    ros::NodeHandle nl("~");

    // setup subscribers
    ros::Subscriber jointStateSub = n.subscribe("joint_states", 100, &MultiContactFootCoordsROS::jointStateCallback, this); // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可

    ros::Subscriber imuSub = n.subscribe("imu", 1, &MultiContactFootCoordsROS::imuCallback, this);

    ros::Subscriber endEffectorsSub = n.subscribe("end_effectors", 1, &MultiContactFootCoordsROS::endEffectorsCallback, this);

    // main loop
    int rate;
    nl.param("rate", rate, 250); // 250 hz
    ros::Rate r(rate);

    unsigned int seq = 0;
    ros::Time stamp = ros::Time::now();
    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      // spin
      ros::spinOnce();

      if( this->isEnabled_ ){
      }

      seq++;
      stamp = now;

      drawObjects();

      r.sleep();
    }

    exit(0);

  }

  void MultiContactFootCoordsROS::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {

  }

  void MultiContactFootCoordsROS::imuCallback(const sensor_msgs::Imu::ConstPtr& msg){

  }

  void MultiContactFootCoordsROS::endEffectorsCallback(const multicontact_controller_msgs::StringArray::ConstPtr& msg) {

  }

  bool MultiContactFootCoordsROS::enableCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    this->isEnabled_ = req.data;
    res.success = true;
    return true;
  }
};
