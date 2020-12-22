#include <multicontact_controller/lib/CnoidBodyUtils/Interaction.h>

#include <cnoid/EigenUtil>
#include <limits>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    Interaction::Interaction()
      : F_(cnoid::Vector6::Zero()),
        F_ref_(cnoid::Vector6::Zero()),
        T_(cnoid::Position::Identity()),
        T_ref_(cnoid::Position::Identity()),
        prev_T_ref_(T_ref_),
        v_ref_(cnoid::Vector3::Zero()),
        w_ref_(cnoid::Vector3::Zero()),
        prev_v_ref_(cnoid::Vector3::Zero()),
        prev_w_ref_(cnoid::Vector3::Zero()),
        prev_v_(cnoid::Vector3::Zero()),
        prev_w_(cnoid::Vector3::Zero()),
        M_p_(100.0),
        D_p_(2000.0),
        K_p_(2000.0),
        M_r_(50.0),
        D_r_(2000.0),
        K_r_(2000.0),
        force_gain_(1,1,1),
        moment_gain_(1,1,1),
        weight_trans_(1e0),
        weight_rot_(1e-2),
        v_limit_(0.1),
        w_limit_(0.174533)
    {
    }

    void Interaction::desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6);
      for(size_t i=0;i<6;i++)A.insert(i,i) = 1.0;
      b.resize(6);
      wa.resize(6);

      v_ref_ = T_ref_.linear().transpose() * (T_ref_.translation() - prev_T_ref_.translation()) / dt_;
      w_ref_ = cnoid::omegaFromRot(prev_T_ref_.linear().transpose() * T_ref_.linear()) / dt_;

      b.head<3>() =
        ((F_.head<3>()-F_ref_.head<3>()).cwiseProduct(force_gain_)
         + (T_ref_.translation() - T_.translation()) * K_p_
         + v_ref_ * D_p_
         + (v_ref_ - prev_v_ref_ + prev_v_) * M_p_ / dt_)
        / (M_p_ / std::pow(dt_,2) + D_p_ / dt_ + K_p_);

      b.tail<3>() =
        ((F_.tail<3>()-F_ref_.tail<3>()).cwiseProduct(moment_gain_)
         + cnoid::omegaFromRot(T_.linear().transpose() * T_ref_.linear()) * K_r_
         + w_ref_ * D_r_
         + (w_ref_ - prev_w_ref_ + prev_w_) * M_r_ / dt_)
        / (M_r_ / std::pow(dt_,2) + D_r_ / dt_ + K_r_);

      for(size_t i=0;i<3;i++){
        if(b[i] > v_limit_ * dt_) b[i] = v_limit_ * dt_;
        if(b[i] < - v_limit_ * dt_) b[i] = - v_limit_ * dt_;
      }
      for(size_t i=3;i<6;i++){
        if(b[i] > w_limit_ * dt_) b[i] = w_limit_ * dt_;
        if(b[i] < - w_limit_ * dt_) b[i] = - w_limit_ * dt_;
      }

      for(size_t i=0;i<3;i++) wa[i] = weight_trans_;
      for(size_t i=3;i<6;i++) wa[i] = weight_rot_;

      //Cはゼロ
      C.resize(0,6);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
      return;
    }

    void Interaction::update(const cnoid::Vector3& v, const cnoid::Vector3& w){
      prev_v_ = v;
      prev_w_ = w;
      prev_v_ref_ = v_ref_;
      prev_w_ref_ = w_ref_;
      prev_T_ref_ = T_ref_;
    }
  }
}