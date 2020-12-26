#include "JointLimitTable.h"

double RobotConfig::JointLimitTable::getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const
{
    double target_angle = target_joint_angle * 180.0 / M_PI; // [rad]=>[deg]
    int int_target_angle = static_cast<int>(std::floor(target_angle));
    int target_range[2] = {int_target_angle, 1+int_target_angle};
    double self_joint_range[2];
    for (size_t i = 0; i < 2; i++) {
        size_t idx = std::min(std::max(target_llimit_angle, target_range[i]), target_ulimit_angle) - target_llimit_angle;
        self_joint_range[i] = (is_llimit_angle ? llimit_table[idx] : ulimit_table[idx]);
    }
    double tmp_ratio = target_angle - int_target_angle;
    return (self_joint_range[0] * (1-tmp_ratio) + self_joint_range[1] * tmp_ratio) * M_PI / 180.0; // [deg]=>[rad]
};

void RobotConfig::readJointLimitTableFromProperties (std::map<const cnoid::Link*, std::shared_ptr<RobotConfig::JointLimitTable> >& joint_limit_tables,
                                                     const cnoid::Body* robot,
                                                     const std::string& prop_string)
{
  size_t limit_table_size = 6; // self_joint_name:target_joint_name:target_min_angle:target_max_angle:min:max

  std::stringstream ss(prop_string);
  std::vector<std::string> item(limit_table_size);

  while (std::getline(ss, item[0], ':')) {
    for(size_t i=1;i<limit_table_size;i++){
      std::getline(ss, item[i], ':');
    }

    const cnoid::Link* self_joint = robot->link(item[0]);
    const cnoid::Link* target_joint = robot->link(item[1]);

    int target_llimit_angle = std::stoi(item[2]);
    int target_ulimit_angle = std::stoi(item[3]);

    std::vector<double> llimit_table;
    std::vector<double> ulimit_table;
    std::stringstream ssl(item[4]);
    std::stringstream ssu(item[5]);
    std::string sl, su;
    while (std::getline(ssl, sl, ',') && std::getline(ssu, su, ',')) {
      llimit_table.push_back(std::stod(sl));
      ulimit_table.push_back(std::stod(su));
    }

    if ( llimit_table.size() != ulimit_table.size() || !target_joint || ! self_joint) {
      std::cerr << "[RobotConfig::readJointLimitTableFromProperties] " << item[0] << ":" << item[1] << " fail" << std::endl;
      continue;
    }

    joint_limit_tables.insert(std::pair<const cnoid::Link*, std::shared_ptr<RobotConfig::JointLimitTable> >
                              (self_joint,
                               std::make_shared<RobotConfig::JointLimitTable>(self_joint,
                                                                              target_joint,
                                                                              target_llimit_angle,
                                                                              target_ulimit_angle,
                                                                              llimit_table,
                                                                              ulimit_table)));
  }
};
