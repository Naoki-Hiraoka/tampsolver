#include <iir_filter/Filter.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <functional>

class JointStateFilter {
public:
  JointStateFilter(const ros::Time& stamp, const std::string& name, const double& init_position, const double& init_velocity, const double& init_effort, const double& position_cutoff_freq, const double& velocity_cutoff_freq, const double& effort_cutoff_freq):
    name_(name),
    stamp_(stamp),
    position_filtered_(init_position),
    velocity_filtered_(init_velocity),
    effort_filtered_(init_effort),
    position_filter_(position_cutoff_freq,0.1,init_position),
    velocity_filter_(velocity_cutoff_freq,0.1,init_velocity),
    effort_filter_(effort_cutoff_freq,0.1,init_effort)
  {
  }
  void passFilter(const ros::Time& stamp, const double& position, const double& velocity, const double& effort) {
    double dt = (stamp - stamp_).toSec();
    if(dt<=0) return;

    position_filtered_ = position_filter_.passFilter(position,dt);
    velocity_filtered_ = velocity_filter_.passFilter(velocity,dt);
    effort_filtered_ = effort_filter_.passFilter(effort,dt);

    stamp_ = stamp;
  }

  std::string& name() { return name_; }
  std::string name() const { return name_; }
  double& position() { return position_filtered_; }
  double position() const {return position_filtered_; }
  double& velocity() {return velocity_filtered_; }
  double velocity() const {return velocity_filtered_; }
  double& effort() {return effort_filtered_; }
  double effort() const {return effort_filtered_; }
private:
  std::string name_;
  ros::Time stamp_;
  double position_filtered_;
  double velocity_filtered_;
  double effort_filtered_;
  iir_filter::FirstOrderLowPassFilter2<double> position_filter_;
  iir_filter::FirstOrderLowPassFilter2<double> velocity_filter_;
  iir_filter::FirstOrderLowPassFilter2<double> effort_filter_;
};


int main(int argc, char** argv){
  ros::init(argc,argv,"joint_states_filter");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  double position_cutoff_freq;
  double velocity_cutoff_freq;
  double effort_cutoff_freq;
  nl.param("position_cutoff_freq", position_cutoff_freq, 1.0); // 1 hz
  nl.param("velocity_cutoff_freq", velocity_cutoff_freq, 1.0); // 1 hz
  nl.param("effort_cutoff_freq", effort_cutoff_freq, 1.0); // 1 hz

  std::map<std::string, std::shared_ptr<JointStateFilter> > jointStateMap;
  ros::Subscriber jointStateSub =
    n.subscribe<sensor_msgs::JointState>("joint_states",
                                         1000, // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可
                                         [&](const sensor_msgs::JointState::ConstPtr& msg){
                                           const ros::Time stamp = msg->header.stamp;
                                           for(size_t i=0;i<msg->name.size();i++){
                                             const std::string& name = msg->name[i];
                                             double position = msg->position.size()==msg->name.size() ? msg->position[i] : 0;
                                             double velocity = msg->velocity.size()==msg->name.size() ? msg->velocity[i] : 0;
                                             double effort = msg->effort.size()==msg->name.size() ? msg->effort[i] : 0;
                                             if(jointStateMap.find(name) == jointStateMap.end()){
                                               jointStateMap[name] = std::make_shared<JointStateFilter>(stamp,
                                                                                                        name,
                                                                                                        position,
                                                                                                        velocity,
                                                                                                        effort,
                                                                                                        position_cutoff_freq,
                                                                                                        velocity_cutoff_freq,
                                                                                                        effort_cutoff_freq);
                                             }else{
                                               jointStateMap[name]->passFilter(stamp,
                                                                               position,
                                                                               velocity,
                                                                               effort);
                                             }
                                           }
                                         });

  ros::Publisher jointStateFilteredPub = n.advertise<sensor_msgs::JointState>("joint_states_filtered", 1000);

  int rate;
  nl.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  unsigned int seq = 0;
  while (ros::ok()) {
    // spin
    ros::spinOnce();

    sensor_msgs::JointState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq;
    for(std::map<std::string, std::shared_ptr<JointStateFilter> >::iterator it=jointStateMap.begin();it!=jointStateMap.end();it++){
      msg.name.push_back(it->second->name());
      msg.position.push_back(it->second->position());
      msg.velocity.push_back(it->second->velocity());
      msg.effort.push_back(it->second->effort());
    }
    jointStateFilteredPub.publish(msg);
    seq++;
    r.sleep();
  }

  return 0;
}
