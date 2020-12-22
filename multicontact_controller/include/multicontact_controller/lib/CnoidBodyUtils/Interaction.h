#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_INTERACTION_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_INTERACTION_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>
#include <multicontact_controller_msgs/InteractionInfo.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    class Interaction {
    public:
      Interaction();

      void desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

      // odom系の座標のref値
      cnoid::Position T_ref() const { return T_ref_; }
      cnoid::Position& T_ref() {return T_ref_; }
      void reset_ref() {
        F_ref_ = F_;
        prev_T_ref_ = T_ref_ = T_;
        v_ref_ = w_ref_ = cnoid::Vector3::Zero();
        prev_v_ref_ = prev_w_ref_ = cnoid::Vector3::Zero();
        prev_v_ = prev_w_ = cnoid::Vector3::Zero();
      }

      void update(const cnoid::Vector3& v, const cnoid::Vector3& w);

      cnoid::Vector6 F() const { return F_;}
      cnoid::Vector6& F() { return F_;}
      cnoid::Vector6 F_ref() const { return F_ref_;}
      cnoid::Vector6& F_ref() { return F_ref_;}
      cnoid::Position T() const { return T_; }
      cnoid::Position& T() {return T_; }
      double dt() const { return dt_;}
      double& dt() { return dt_;}

      double M_p() const { return M_p_;}
      double& M_p() { return M_p_;}
      double D_p() const { return D_p_;}
      double& D_p() { return D_p_;}
      double K_p() const { return K_p_;}
      double& K_p() { return K_p_;}
      double M_r() const { return M_r_;}
      double& M_r() { return M_r_;}
      double D_r() const { return D_r_;}
      double& D_r() { return D_r_;}
      double K_r() const { return K_r_;}
      double& K_r() { return K_r_;}
      cnoid::Vector3 force_gain() const { return force_gain_;}
      cnoid::Vector3& force_gain() { return force_gain_;}
      cnoid::Vector3 moment_gain() const { return moment_gain_;}
      cnoid::Vector3& moment_gain() { return moment_gain_;}
      double weight_trans() const { return weight_trans_;}
      double& weight_trans() { return weight_trans_;}
      double weight_rot() const { return weight_rot_;}
      double& weight_rot() { return weight_rot_;}
      double v_limit() const { return v_limit_;}
      double& v_limit() { return v_limit_;}
      double w_limit() const { return w_limit_;}
      double& w_limit() { return w_limit_;}
    protected:
      cnoid::Vector6 F_;//local系
      cnoid::Vector6 F_ref_;//local系
      cnoid::Position T_;//world系
      cnoid::Position T_ref_;//world系
      double dt_;

      cnoid::Vector3 v_ref_;//local系
      cnoid::Vector3 w_ref_;//local系
      cnoid::Position prev_T_ref_;//world系
      cnoid::Vector3 prev_v_ref_;//local系
      cnoid::Vector3 prev_w_ref_;//local系
      cnoid::Vector3 prev_v_;//local系
      cnoid::Vector3 prev_w_;//local系

      // params
      double M_p_;
      double D_p_;
      double K_p_;
      double M_r_;
      double D_r_;
      double K_r_;
      cnoid::Vector3 force_gain_;
      cnoid::Vector3 moment_gain_;
      double weight_trans_;
      double weight_rot_;
      double v_limit_;
      double w_limit_;

    };

    bool loadInteractionFromInfo(const multicontact_controller_msgs::InteractionInfo& msg, std::shared_ptr<Interaction>& contact);
  }
}

#endif
