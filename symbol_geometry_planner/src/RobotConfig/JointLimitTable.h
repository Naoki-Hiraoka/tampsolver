#ifndef JOINT_LIMIT_TABLE_H
#define JOINT_LIMIT_TABLE_H

#include <cnoid/BodyItem>
#include <map>
#include <iostream>

namespace RobotConfig {
    // JointLimitTable for one joint
    //   self_joint   : a joint to obtain llimit and ulimit from this class.
    //   target_joint : self_joint's limit is difference for target_joint's joint angle.
    class JointLimitTable {
    public:
      JointLimitTable (const cnoid::Link* _self_joint, const cnoid::Link* _target_joint,
                       const int _target_llimit_angle, const int _target_ulimit_angle,
                       const std::vector<double>& _llimit_table, const std::vector<double>& _ulimit_table)
        : self_joint(_self_joint), target_joint(_target_joint), target_llimit_angle(_target_llimit_angle), target_ulimit_angle(_target_ulimit_angle), llimit_table(_llimit_table), ulimit_table(_ulimit_table) {};
      ~JointLimitTable() {};
      const cnoid::Link* getSelfJoint () const { return self_joint; };
      const cnoid::Link* getTargetJoint () const { return target_joint; };
      double getLlimit () const // [rad]
      {
        return getInterpolatedLimitAngle(target_joint->q(), true); // [rad]
      };
      double getUlimit () const // [rad]
      {
        return getInterpolatedLimitAngle(target_joint->q(), false); // [rad]
      };
    private:
      const cnoid::Link* self_joint;
      const cnoid::Link* target_joint;
      int target_llimit_angle, target_ulimit_angle; // llimit and ulimit angle [deg] for target_joint
      std::vector<double> llimit_table, ulimit_table; // Tables for self_joint's llimit and ulimit
      double getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const;
    };

  void readJointLimitTableFromProperties (std::map<const cnoid::Link*, std::shared_ptr<RobotConfig::JointLimitTable> >& joint_mm_tables,
                                            cnoid::BodyItemPtr robot,
                                            const std::string& prop_string);
};

#endif
