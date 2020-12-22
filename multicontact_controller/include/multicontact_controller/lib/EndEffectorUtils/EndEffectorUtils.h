#ifndef MULTICONTACT_CONTROLLER_ENDEFFECTOR_UTILS_H
#define MULTICONTACT_CONTROLLER_ENDEFFECTOR_UTILS_H

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <multicontact_controller_msgs/StringArray.h>
#include <multicontact_controller_msgs/EndEffectorInfo.h>
#include <cnoid/Body>
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>
#include <eigen_conversions/eigen_msg.h>

namespace multicontact_controller {
  namespace endeffectorutils {
    template <typename T, class... Args>
    void stringArrayToEndEffectors(const multicontact_controller_msgs::StringArray::ConstPtr& msg, std::map<std::string,std::shared_ptr<T> >& endEffectors, Args&... args){
      // 消滅したEndEffectorを削除
      for(typename std::map<std::string,std::shared_ptr<T> >::iterator it = endEffectors.begin(); it != endEffectors.end(); ) {
        if (std::find_if(msg->strings.begin(),msg->strings.end(),[&](std::string x){return x==it->first;}) == msg->strings.end()) {
          it = endEffectors.erase(it);
        }
        else {
          ++it;
        }
      }

      // 増加したEndEffectorの反映
      for(size_t i=0;i<msg->strings.size();i++){
        if(endEffectors.find(msg->strings[i])==endEffectors.end()){
          endEffectors[msg->strings[i]] = std::make_shared<T>(msg->strings[i], args...);
        }
      }
    }

    class EndEffectorClient {
    public:
      EndEffectorClient(const std::string& name)
        : name_(name),
          state_("NOT_CARED"),
          prev_state_("NOT_CARED")
      {
        ros::NodeHandle n;
        infoSub_ = n.subscribe(name_ + "/info", 1, &EndEffectorClient::infoCallback, this);
        stateSub_ = n.subscribe(name_ + "/state", 100, &EndEffectorClient::stateCallback, this);
      }
      void infoCallback(const multicontact_controller_msgs::EndEffectorInfo::ConstPtr& msg){
        info_ = msg;
        onInfoUpdated();
      };
      virtual void onInfoUpdated() {return;};
      void stateCallback(const std_msgs::String::ConstPtr& msg){
        prev_state_ = state_;
        state_ = msg->data;
        onStateUpdated();
      }
      virtual void onStateUpdated() {return;}

      // infoがしっかり届いているか
      virtual bool isValid() {return bool(info_);}

      std::string& name() {return name_;}
      std::string name() const {return name_;}
      std::string& state() {return state_;}
      std::string state() const {return state_;}
    protected:
      std::string name_;
      std::string state_;
      std::string prev_state_;
      multicontact_controller_msgs::EndEffectorInfo::ConstPtr info_;

      ros::Subscriber infoSub_;
      ros::Subscriber stateSub_;
    };

    void updateContactPointFromInfo(cnoid::Body* robot, std::shared_ptr<cnoidbodyutils::ContactPoint> contactPoint, const multicontact_controller_msgs::EndEffectorInfo& info);

  };
};

#endif
