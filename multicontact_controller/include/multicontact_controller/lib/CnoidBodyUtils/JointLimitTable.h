#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_JOINT_LIMIT_TABLE_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_JOINT_LIMIT_TABLE_H

#include <cnoid/Body>
#include <map>
#include <memory>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    // JointLimitTable for one joint
    //   self_joint   : a joint to obtain llimit and ulimit from this class.
    //   target_joint : self_joint's limit is difference for target_joint's joint angle.
    class JointLimitTable {
    public:
      JointLimitTable (const cnoid::Link* self_joint, const cnoid::Link* target_joint,
                       const int target_llimit_angle, const int target_ulimit_angle,
                       const std::vector<double>& llimit_table, const std::vector<double>& ulimit_table)
        : self_joint_(self_joint),
          target_joint_(target_joint),
          target_llimit_angle_(target_llimit_angle),
          target_ulimit_angle_(target_ulimit_angle),
          llimit_table_(llimit_table),
          ulimit_table_(ulimit_table) {
      };
      const cnoid::Link* getSelfJoint () const { return self_joint_; };
      const cnoid::Link* getTargetJoint () const { return target_joint_; };
      double getLlimit (double target_joint_angle) const // [rad]
      {
        return getInterpolatedLimitAngle(target_joint_angle, true); // [rad]
      };
      double getLlimit () const
      {
        return getLlimit(target_joint_->q());
      }
      double getUlimit (double target_joint_angle) const // [rad]
      {
        return getInterpolatedLimitAngle(target_joint_angle, false); // [rad]
      };
      double getUlimit () const
      {
        return getUlimit(target_joint_->q());
      }
    private:
      const cnoid::Link* self_joint_;
      const cnoid::Link* target_joint_;
      int target_llimit_angle_, target_ulimit_angle_; // llimit and ulimit angle [deg] for target_joint
      std::vector<double> llimit_table_, ulimit_table_; // Tables for self_joint's llimit and ulimit
      double getInterpolatedLimitAngle (const double target_joint_angle, const bool is_llimit_angle) const;
    };

  std::shared_ptr<JointLimitTable> readJointLimitTableFromProperty (const cnoid::Body* robot,
                                                                                 const std::string& prop_string);
  };
};

#endif
