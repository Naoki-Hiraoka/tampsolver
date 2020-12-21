#include <multicontact_controller/lib/CnoidBodyUtils/JointLimitTable.h>
#include <ros/ros.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    double JointLimitTable::getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const {
      double target_angle = target_joint_angle * 180.0 / M_PI; // [rad]=>[deg]
      int int_target_angle = static_cast<int>(std::floor(target_angle));
      int target_range[2] = {int_target_angle, 1+int_target_angle};
      double self_joint_range[2];
      for (size_t i = 0; i < 2; i++) {
        size_t idx = std::min(std::max(target_llimit_angle_, target_range[i]), target_ulimit_angle_) - target_llimit_angle_;
        self_joint_range[i] = (is_llimit_angle ? llimit_table_[idx] : ulimit_table_[idx]);
      }
      double tmp_ratio = target_angle - int_target_angle;
      return (self_joint_range[0] * (1-tmp_ratio) + self_joint_range[1] * tmp_ratio) * M_PI / 180.0; // [deg]=>[rad]
    };

    std::shared_ptr<JointLimitTable> readJointLimitTableFromParam (const cnoid::Body* robot,
                                                                   const std::string& ns) {
      ros::NodeHandle n;
      if(!n.hasParam(ns+"/limit_table")) return nullptr;

      std::string param;
      n.getParam(ns+"/limit_table", param);

      std::stringstream ss(param);

      size_t limit_table_size = 6; // self_joint_name:target_joint_name:target_min_angle:target_max_angle:min:max

      std::vector<std::string> items;
      {
        std::string item;
        while (std::getline(ss, item, ':')) {
          items.push_back(item);
        }
      }

      if(items.size() != 6) {
        ROS_ERROR("size of %s in %s != 6",param.c_str(),(ns+"/limit_table").c_str());
        return nullptr;
      }

      const cnoid::Link* self_joint = robot->link(items[0]);
      const cnoid::Link* target_joint = robot->link(items[1]);

      int target_llimit_angle = std::stoi(items[2]);
      int target_ulimit_angle = std::stoi(items[3]);

      std::vector<double> llimit_table;
      std::vector<double> ulimit_table;
      std::stringstream ssl(items[4]);
      std::stringstream ssu(items[5]);
      std::string sl, su;
      while (std::getline(ssl, sl, ',') && std::getline(ssu, su, ',')) {
        llimit_table.push_back(std::stod(sl));
        ulimit_table.push_back(std::stod(su));
      }

      if ( llimit_table.size() != ulimit_table.size() || !target_joint || ! self_joint) {
        ROS_ERROR("[readJointLimitTableFromProperties] %s : %s fail", items[0].c_str(),items[1].c_str());
        return nullptr;
      }

      return std::make_shared<JointLimitTable>(self_joint,
                                               target_joint,
                                               target_llimit_angle,
                                               target_ulimit_angle,
                                               llimit_table,
                                               ulimit_table);
    }
  };
};
