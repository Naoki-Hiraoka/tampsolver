#ifndef PWT_CONTROLLER_H
#define PWT_CONTROLLER_H

#include <cnoid/Body>
#include <memory>
#include <Eigen/Sparse>
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {
  class ContactPointPWTC: public cnoidbodyutils::ContactPoint {
  public:
    // odom系の座標のref値
    cnoid::Position T_ref() const { return T_ref_; }
    cnoid::Position& T_ref() {return T_ref_; }
    cnoid::Position prev_T_ref() const { return prev_T_ref_; }
    cnoid::Position& prev_T_ref() {return prev_T_ref_; }
    cnoid::Position prev_prev_T_ref() const { return prev_prev_T_ref_; }
    cnoid::Position& prev_prev_T_ref() {return prev_prev_T_ref_; }

    std::shared_ptr<cnoidbodyutils::Contact> contact() const { return contact_;}
    std::shared_ptr<cnoidbodyutils::Contact>& contact() { return contact_;}
  private:
    cnoid::Position T_ref_;
    cnoid::Position prev_T_ref_;
    cnoid::Position prev_prev_T_ref_;

    std::shared_ptr<cnoidbodyutils::Contact> contact_;
  };

  class JointInfo {
  public:
  protected:
    std::string name_;

    double coil_temperature_limit_;
    double housing_temperature_;
    double coil_temperature_;
    double maximum_effort_soft_;
    double maximum_effort_hard_;
    double balance_effort_;
    double remaining_time_;

    double pgain_;
    double dgain_;

    std::shared_ptr<cnoidbodyutils::JointLimitTable> jointLimitTable_;
  };

  class PWTController {
  public:
    PWTController(){
    }

    bool calcPWTControl(cnoid::Body* robot);
  private:

    // cache
  };

};

#endif
