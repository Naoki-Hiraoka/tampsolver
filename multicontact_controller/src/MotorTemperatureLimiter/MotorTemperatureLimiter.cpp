#include <ros/ros.h>
#include <multicontact_controller_msgs/MotorTemperatureState.h>
#include <hrpsys_ros_bridge/MotorStates.h>
#include <hrpsys_ros_bridge/OpenHRP_SoftErrorLimiterService_setServoErrorLimit.h>
#include <cmath>

class MotorTemperatureLimiterImpl {
public:
  bool isValid(){
    if(hardware_pgain_ ==0 || pgain_ ==0 || safety_factor_ <= 1 || temperature_thre_ <= 0 || max_error_ < 0 || max_vel_ < 0) return false;
    return true;
  }

  bool execute(ros::ServiceClient& client, double dt){
    if(!isValid()) return false;

    if(!limit_active_){
      if(coil_temperature_limit_ + temperature_thre_ < temperature_) limit_active_ = true;
    }

    if(limit_active_){
      if(coil_temperature_limit_ < temperature_){
        target_error_ = std::min(max_error_,
                                 std::abs(balance_effort_) / safety_factor_ / std::abs(pgain_*hardware_pgain_));
      }else{
        target_error_ = max_error_;
        limit_active_ = false;
      }
    }

    if(target_error_ != prev_error_){
      hrpsys_ros_bridge::OpenHRP_SoftErrorLimiterService_setServoErrorLimit srv;
      srv.request.name = name_;
      double limit = std::max(std::min(target_error_,prev_error_ + max_vel_*dt),prev_error_ - max_vel_*dt);
      srv.request.limit = limit;
      ROS_INFO("Temperature %s is large (%f degree)! reduce errorlimit", name_.c_str(), temperature_);
      if (!client.call(srv)) ROS_WARN("call setServoErrorLimit failed");
      else prev_error_ = limit;
    }

    return true;
  }

  std::string name() const {return name_;}
  std::string& name() {return name_;}
  double temperature() const {return temperature_;}
  double& temperature() {return temperature_;}
  double balance_effort() const {return balance_effort_;}
  double& balance_effort() {return balance_effort_;}
  double coil_temperature_limit() const {return coil_temperature_limit_;}
  double& coil_temperature_limit() {return coil_temperature_limit_;}
  double pgain() const {return pgain_;}
  double& pgain() {return pgain_;}
  double hardware_pgain() const {return hardware_pgain_;}
  double& hardware_pgain() {return hardware_pgain_;}

  double temperature_thre() const {return temperature_thre_;}
  double& temperature_thre() {return temperature_thre_;}
  double safety_factor() const {return safety_factor_;}
  double& safety_factor() {return safety_factor_;}
  double max_error() const {return max_error_;}
  double& max_error() {return max_error_;}
  double max_vel() const {return max_vel_;}
  double& max_vel() {return max_vel_;}

private:
  std::string name_;
  double temperature_;
  double balance_effort_;
  double coil_temperature_limit_;
  double pgain_;
  double hardware_pgain_;

  double temperature_thre_;
  double safety_factor_;
  double max_error_;
  double max_vel_;
  bool limit_active_;

  double prev_error_;
  double target_error_;
};

int main(int argc, char** argv){
  ros::init(argc,argv,"motor_temperature_limiter");
  ros::NodeHandle n;
  ros::NodeHandle nl("~");

  double temperature_thre;
  nl.param("temperature_thre", temperature_thre, 5.0);

  double safety_factor;
  nl.param("safety_factor", safety_factor, 1.2);

  double max_error;
  nl.param("max_error", max_error, 0.2 - 0.02);//same as SoftErrorLimiter

  double max_vel;
  nl.param("max_vel", max_vel, 0.1);

  std::map<std::string,std::shared_ptr<MotorTemperatureLimiterImpl> > limiterMap;

  ros::Subscriber motorTemperatureStateSub =
    n.subscribe<multicontact_controller_msgs::MotorTemperatureState>
    ("motor_temperature_states",
     1000, // 一部しか含まないmotor_temperature_statesにも対応するため、バッファは1(最新のみ)では不可
     [&](const multicontact_controller_msgs::MotorTemperatureState::ConstPtr& msg){
      if(msg->name.size() != msg->coil_temperature.size() ||
         msg->name.size() != msg->coil_temperature_limit.size() ||
         msg->name.size() != msg->balance_effort.size() ) return;
      for(size_t i=0;i<msg->name.size();i++){
        if(limiterMap.find(msg->name[i]) == limiterMap.end()){
          std::shared_ptr<MotorTemperatureLimiterImpl> impl = std::make_shared<MotorTemperatureLimiterImpl>();
          impl->name() = msg->name[i];
          impl->temperature_thre() = temperature_thre;
          impl->safety_factor() = safety_factor;
          impl->max_error() = max_error;
          impl->max_vel() = max_vel;
          n.param("joint_config/"+msg->name[i]+"/hardware_pgain",impl->hardware_pgain(),1.0);
          limiterMap[msg->name[i]] = impl;
        }
        std::shared_ptr<MotorTemperatureLimiterImpl> impl = limiterMap[msg->name[i]];
        impl->temperature() = msg->coil_temperature[i];
        impl->coil_temperature_limit() = msg->coil_temperature_limit[i];
        impl->balance_effort() = msg->balance_effort[i];
      }
     });

  ros::Subscriber motorStatesSub =
    n.subscribe<hrpsys_ros_bridge::MotorStates>
    ("motor_states",
     1000, // 一部しか含まないmotor_statesにも対応するため、バッファは1(最新のみ)では不可
     [&](const hrpsys_ros_bridge::MotorStates::ConstPtr& msg){
      if(msg->name.size() != msg->pgain.size()) return;
      for(size_t i=0;i<msg->name.size();i++){
        if(limiterMap.find(msg->name[i]) != limiterMap.end()){
          limiterMap[msg->name[i]]->pgain() = msg->pgain[i];
        }
      }
     });

  ros::ServiceClient client = n.serviceClient<hrpsys_ros_bridge::OpenHRP_SoftErrorLimiterService_setServoErrorLimit>("setServoErrorLimit");
  if(!client.waitForExistence(ros::Duration(10.0))){
    ROS_ERROR("service setServoErrorLimit not found");
    return 1;
  }


  int rate;
  nl.param("rate", rate, 3); // 3 hz
  ros::Rate r(rate);

  ros::Time stamp = ros::Time::now();
  while (ros::ok()) {
    // spin
    ros::spinOnce();

    ros::Time now = ros::Time::now();
    double dt = (now - stamp).toSec();

    for(std::map<std::string, std::shared_ptr<MotorTemperatureLimiterImpl> >::iterator it=limiterMap.begin();it!=limiterMap.end();it++){
      it->second->execute(client,dt);
    }

    stamp = now;
    r.sleep();
  }

  return 0;
}
