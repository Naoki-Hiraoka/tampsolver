#include <multicontact_controller/MotorTemperatureEstimator/MotorTemperatureEstimator.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <hrpsys_ros_bridge/MotorStates.h>
#include <multicontact_controller_msgs/MotorTemperatureState.h>

class MotorTemperatureEstimatorImpl {
public:
  bool estimate(const double& tau, const ros::Time& stamp) {
    if(stamp_ == ros::Time(0)){
      stamp_ = stamp;
      return true;
    }

    double dt = (stamp - stamp_).toSec();
    if(dt<=0) return false;

    estimator_.estimate(tau,dt);

    stamp_ = stamp;
  }

  bool loadParam(ros::NodeHandle& n, const std::string& ns, double Tair) {
    if(!n.hasParam(ns+"/name")){
      ROS_WARN("rosparam %s not found",(ns+"/name").c_str());
      return false;
    }
    n.getParam(ns+"/name",name_);

    if(!n.hasParam(ns+"/thermal_params/Re")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/Re").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/Re",estimator_.Re());

    if(!n.hasParam(ns+"/thermal_params/K")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/K").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/K",estimator_.K());

    if(!n.hasParam(ns+"/thermal_params/Ccoil")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/Ccoil").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/Ccoil",estimator_.Ccoil());

    if(!n.hasParam(ns+"/thermal_params/Chousing")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/Chousing").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/Chousing",estimator_.Chousing());

    if(!n.hasParam(ns+"/thermal_params/R1")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/R1").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/R1",estimator_.R1());

    if(!n.hasParam(ns+"/thermal_params/R2")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/R2").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/R2",estimator_.R2());

    if(!n.hasParam(ns+"/thermal_params/Tlimit")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/Tlimit").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/Tlimit",estimator_.Tlimit());

    if(!n.hasParam(ns+"/thermal_params/tsoft")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/tsoft").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/tsoft",estimator_.tsoft());

    if(!n.hasParam(ns+"/thermal_params/thard")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/thard").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/thard",estimator_.thard());

    if(!n.hasParam(ns+"/thermal_params/TempPredParams")){
      ROS_WARN("rosparam %s not found",(ns+"/thermal_params/TempPredParams").c_str());
      return false;
    }
    n.getParam(ns+"/thermal_params/TempPredParams",estimator_.TempPredParams());

    n.param(ns+"/thermal_params/maxRemainingTime",estimator_.maxRemainingTime(),600.0);
    n.param(ns+"/thermal_params/remainingTimeStep",estimator_.remainingTimeStep(),5.0);

    estimator_.Tcoil() = Tair;
    estimator_.Thousing() = Tair;
    estimator_.Tair() = Tair;

    if(!estimator_.isValid()){
      ROS_WARN("parameter not valid");
      return false;
    }

    return true;
  }

  std::string name() const {return name_;}
  std::string& name() {return name_;}
  double Tcoil() const {return estimator_.Tcoil();}
  double& Tcoil() {return estimator_.Tcoil();}
  double Thousing() const {return estimator_.Thousing();}
  double& Thousing() {return estimator_.Thousing();}
  double Tlimit() const {return estimator_.Tlimit();}
  double& Tlimit() {return estimator_.Tlimit();}
  double tauMaxsoft() const {return estimator_.tauMaxsoft();}
  double& tauMaxsoft() {return estimator_.tauMaxsoft();}
  double tauMaxhard() const {return estimator_.tauMaxhard();}
  double& tauMaxhard() {return estimator_.tauMaxhard();}
  double tauBalance() const {return estimator_.tauBalance();}
  double& tauBalance() {return estimator_.tauBalance();}
  double remainingTime() const {return estimator_.remainingTime();}
  double& remainingTime() {return estimator_.remainingTime();}

private:
  multicontact_controller::MotorTemperatureEstimator estimator_;
  std::string name_;
  ros::Time stamp_ = ros::Time(0);
};

int main(int argc, char** argv){
  ros::init(argc,argv,"motor_temperature_estimator");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  double Tair;
  n.param("Tair", Tair, 25.0);

  std::vector<std::string> joints;
  n.getParam("joints", joints); // link of joints to estimate

  std::map<std::string,std::shared_ptr<MotorTemperatureEstimatorImpl> > motorTemperatureEstimators;

  for(size_t i=0;i<joints.size();i++){
    std::string ns = "joint_config/" + joints[i];
    std::shared_ptr<MotorTemperatureEstimatorImpl> motorTemperatureEstimator = std::make_shared<MotorTemperatureEstimatorImpl>();
    motorTemperatureEstimator->name() = joints[i];
    if(motorTemperatureEstimator->loadParam(n,ns,Tair)){
      motorTemperatureEstimators[joints[i]] = motorTemperatureEstimator;
    }
  }

  ros::Subscriber jointStateSub =
    n.subscribe<sensor_msgs::JointState>("joint_states",
                                         1000, // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可
                                         [&](const sensor_msgs::JointState::ConstPtr& msg){
                                           const ros::Time stamp = msg->header.stamp;
                                           for(size_t i=0;i<msg->name.size();i++){
                                             const std::string& name = msg->name[i];
                                             double effort = msg->effort.size()==msg->name.size() ? msg->effort[i] : 0;
                                             if(motorTemperatureEstimators.find(name) != motorTemperatureEstimators.end()){
                                               motorTemperatureEstimators[name]->estimate(effort,stamp);
                                             }
                                           }
                                         });

  ros::Subscriber motorStateSub =
    n.subscribe<hrpsys_ros_bridge::MotorStates>("motor_states",
                                               1000, // 一部しか含まないjoint_statesにも対応するため、バッファは1(最新のみ)では不可
                                               [&](const hrpsys_ros_bridge::MotorStates::ConstPtr& msg){
                                                 if(msg->name.size() != msg->temperature.size()) return;
                                                 for(size_t i=0;i<msg->name.size();i++){
                                                   const std::string& name = msg->name[i];
                                                   if(msg->temperature[i]==0)continue;// temperatureが0の関節は無視する
                                                   double temperature = msg->temperature[i];
                                                   if(motorTemperatureEstimators.find(name) != motorTemperatureEstimators.end()){
                                                     motorTemperatureEstimators[name]->Thousing() = temperature;
                                                   }
                                                 }
                                               });

  ros::Publisher motorTemperatureStatePub = n.advertise<multicontact_controller_msgs::MotorTemperatureState>("motor_temperature_states", 1000);

  int rate;
  nl.param("rate", rate, 250); // 250 hz
  ros::Rate r(rate);

  unsigned int seq = 0;
  while (ros::ok()) {
    // spin
    ros::spinOnce();

    multicontact_controller_msgs::MotorTemperatureState msg;
    msg.header.stamp = ros::Time::now();
    msg.header.seq = seq;
    for(std::map<std::string, std::shared_ptr<MotorTemperatureEstimatorImpl> >::iterator it=motorTemperatureEstimators.begin();it!=motorTemperatureEstimators.end();it++){
      msg.name.push_back(it->second->name());
      msg.coil_temperature_limit.push_back(it->second->Tlimit());
      msg.housing_temperature.push_back(it->second->Thousing());
      msg.coil_temperature.push_back(it->second->Tcoil());
      msg.maximum_effort_soft.push_back(it->second->tauMaxsoft());
      msg.maximum_effort_hard.push_back(it->second->tauMaxhard());
      msg.balance_effort.push_back(it->second->tauBalance());
      msg.remaining_time.push_back(it->second->remainingTime());
    }
    motorTemperatureStatePub.publish(msg);
    seq++;
    r.sleep();
  }

  return 0;
}
