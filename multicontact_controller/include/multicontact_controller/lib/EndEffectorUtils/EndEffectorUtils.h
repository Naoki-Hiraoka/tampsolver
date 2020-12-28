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

    // TはContactPoint
    template<typename T>
    void updateContactPointFromInfo(cnoid::Body* robot, std::shared_ptr<T> contactPoint, const multicontact_controller_msgs::EndEffectorInfo& info){
      const std::string& linkname = info.header.frame_id;
      cnoid::Link* link = cnoidbodyutils::getLinkFromURDFlinkName(robot,linkname);
      if(!link) {
        ROS_WARN("Link '%s' is not found in %s",linkname.c_str(),robot->name().c_str());
      }

      contactPoint->parent() = link;
      cnoid::Vector3 p;
      tf::vectorMsgToEigen(info.transform.translation,p);
      contactPoint->T_local().translation() = p;
      Eigen::Quaterniond q;
      tf::quaternionMsgToEigen(info.transform.rotation,q);
      contactPoint->T_local().linear() = q.normalized().toRotationMatrix();
    }

    template<typename T>
    void vectorToString(const std::vector<T>& inVector, std::string& outString){
      std::stringstream ss;
      for(size_t i=0;i<inVector.size();i++){
        ss << inVector[i];
        if(i != inVector.size() -1) ss << ",";
      }
      outString = ss.str();
    }

    template<typename T>
    void stringToVector(const std::string& inString, std::vector<T>& outVector){
      outVector.clear();
      std::stringstream ss(inString);
      std::string s;
      while(std::getline(ss,s,',')){
        std::stringstream ss_(s);
        T s_;
        ss_ >> s_;
        outVector.push_back(s_);
      }
    }

  };
};

#endif
