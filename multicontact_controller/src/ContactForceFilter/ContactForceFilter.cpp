#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

#include <cnoid/Body>
#include <iir_filter/Filter.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <functional>

class EndEffectorCFFROS: public multicontact_controller::endeffectorutils::EndEffectorClient {
  public:
  EndEffectorCFFROS(const std::string& name, double cutoff_freq)
    : multicontact_controller::endeffectorutils::EndEffectorClient::EndEffectorClient(name),
    cutoff_freq_(cutoff_freq),
    force_filter_(cutoff_freq_,0.1,cnoid::Vector6::Zero())
  {
    ros::NodeHandle nh;
    forceFilteredPub_ = nh.advertise<geometry_msgs::WrenchStamped>(name_ + "/force_filtered", 100);
    forceSub_ = nh.subscribe(name_ + "/force", 1, &EndEffectorCFFROS::forceCallback, this);
    stamp_ = ros::Time::now();
  }
  void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg){
    cnoid::Vector6 F;
    tf::wrenchMsgToEigen(msg->wrench,F);

    double dt = (msg->header.stamp - stamp_).toSec();
    if(dt<=0) return;

    cnoid::Vector6 FFiltered = force_filter_.passFilter(F,dt);

    geometry_msgs::WrenchStamped msgFiltered(*msg);
    tf::wrenchEigenToMsg(FFiltered,msgFiltered.wrench);
    forceFilteredPub_.publish(msgFiltered);
    stamp_ = msg->header.stamp;
  };

private:
  ros::Subscriber forceSub_;
  ros::Publisher forceFilteredPub_;
  double cutoff_freq_;
  iir_filter::FirstOrderLowPassFilter2<cnoid::Vector6> force_filter_;
  ros::Time stamp_;
};


int main(int argc, char** argv){
  ros::init(argc,argv,"contact_force_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  double cutoff_freq;
  pnh.param("cutoff_freq", cutoff_freq, 1.0); // 1 hz

  std::map<std::string, std::shared_ptr<EndEffectorCFFROS> > endEffectors;
  ros::Subscriber endEffectorsSub =
    nh.subscribe<multicontact_controller_msgs::StringArray>
    ("end_effectors",
     1,
     [&](const multicontact_controller_msgs::StringArray::ConstPtr& msg) {
      multicontact_controller::endeffectorutils::stringArrayToEndEffectors(msg,endEffectors, cutoff_freq);
     });

  double rate;
  pnh.param("rate", rate, 250.0); // 250 hz
  ros::Rate r(rate);

  while (ros::ok()) {
    // spin
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}
