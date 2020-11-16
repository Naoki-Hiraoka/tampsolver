#include <iir_filter/Filter.h>
#include <ros/ros.h>
#include <geometry_msgs/WrenchStamped.h>
#include <eigen_conversions/eigen_msg.h>
#include <functional>

class ForceSensorFilter {
public:
  ForceSensorFilter(const ros::Time& stamp, const std::string& name, const Eigen::Matrix<double, 6, 1>& init_wrench, const double& cutoff_freq):
    name_(name),
    stamp_(stamp),
    wrench_filtered_(init_wrench),
    wrench_filter_(cutoff_freq,0.1,init_wrench)
  {
  }
  void passFilter(const ros::Time& stamp, const Eigen::Matrix<double, 6, 1>& wrench) {
    double dt = (stamp - stamp_).toSec();
    if(dt<=0) return;

    wrench_filtered_ = wrench_filter_.passFilter(wrench,dt);

    stamp_ = stamp;
  }

  std::string& name() { return name_; }
  std::string name() const { return name_; }
  Eigen::Matrix<double, 6, 1>& wrench() {return wrench_filtered_; }
  Eigen::Matrix<double, 6, 1> wrench() const {return wrench_filtered_; }
private:
  std::string name_;
  ros::Time stamp_;
  Eigen::Matrix<double, 6, 1> wrench_filtered_;
  iir_filter::FirstOrderLowPassFilter2<Eigen::Matrix<double, 6, 1>> wrench_filter_;
};


int main(int argc, char** argv){
  ros::init(argc,argv,"force_sensor_filter");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  double cutoff_freq;
  nl.param("cutoff_freq", cutoff_freq, 1.0); // 1 hz
  std::vector<std::string> forceSensors;
  nl.getParam("force_sensors", forceSensors);

  std::map<std::string, std::shared_ptr<ForceSensorFilter> > forceSensorFilterMap;
  std::map<std::string, ros::Publisher> forceSensorPub;
  std::map<std::string, ros::Subscriber> forceSensorSub;
  for(size_t i=0;i<forceSensors.size();i++){
    forceSensorFilterMap[forceSensors[i]] = std::make_shared<ForceSensorFilter>(ros::Time::now(),forceSensors[i],Eigen::Matrix<double, 6, 1>::Zero(),cutoff_freq);
    forceSensorPub[forceSensors[i]] = n.advertise<geometry_msgs::WrenchStamped>(forceSensors[i]+"_filtered", 1000);
    forceSensorSub[forceSensors[i]]
      = n.subscribe<geometry_msgs::WrenchStamped>(forceSensors[i],
                                                  1,
                                                  [&,i] // iのみコピーで渡す
                                                  (const geometry_msgs::WrenchStamped::ConstPtr& msg){
                                                    Eigen::Matrix<double, 6, 1> wrench;
                                                    tf::wrenchMsgToEigen(msg->wrench,wrench);
                                                    forceSensorFilterMap[forceSensors[i]]->passFilter(msg->header.stamp,wrench);
                                                    geometry_msgs::WrenchStamped newmsg;
                                                    newmsg.header = msg->header;
                                                    tf::wrenchEigenToMsg(forceSensorFilterMap[forceSensors[i]]->wrench(),newmsg.wrench);
                                                    forceSensorPub[forceSensors[i]].publish(newmsg);
                                                  });
  }

  int rate;
  nl.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  while (ros::ok()) {
    // spin
    ros::spinOnce();

    r.sleep();
  }

  return 0;
}
