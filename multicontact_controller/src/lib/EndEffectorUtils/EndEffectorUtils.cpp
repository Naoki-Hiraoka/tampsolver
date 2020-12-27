#include <multicontact_controller/lib/EndEffectorUtils/EndEffectorUtils.h>

namespace multicontact_controller{
  namespace endeffectorutils{
    void updateContactPointFromInfo(cnoid::Body* robot, std::shared_ptr<cnoidbodyutils::ContactPoint> contactPoint, const multicontact_controller_msgs::EndEffectorInfo& info){
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

    void vectorToString(const std::vector<std::string>& stringVector, std::string& outString){
      std::stringstream ss;
      for(size_t i=0;i<stringVector.size();i++){
        ss << stringVector[i];
        if(i != stringVector.size() -1) ss << ",";
      }
      outString = ss.str();
    }
    void stringToVector(const std::string& inString, std::vector<std::string>& stringVector){
      stringVector.clear();
      std::stringstream ss(inString);
      std::string s;
      while(std::getline(ss,s,',')){
        stringVector.push_back(s);
      }
    }
  }
}
