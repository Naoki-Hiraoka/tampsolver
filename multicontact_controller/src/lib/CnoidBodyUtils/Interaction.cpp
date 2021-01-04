#include <multicontact_controller/lib/CnoidBodyUtils/Interaction.h>
#include <multicontact_controller/lib/CnoidBodyUtils/Drawing.h>

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
        K_p_(4000.0),
        M_r_(50.0),
        D_r_(1000.0),
        K_r_(2000.0),
        force_gain_(1,1,1),
        moment_gain_(1,1,1),
        v_limit_(0.1),
        w_limit_(0.174533)
    {
    }

    void Interaction::desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A_A, cnoid::VectorX& b_A, cnoid::VectorX& wa_A, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_A, cnoid::VectorX& dl_A, cnoid::VectorXd& du_A, cnoid::VectorX& wc_A,
                                                Eigen::SparseMatrix<double,Eigen::RowMajor>& A_B, cnoid::VectorX& b_B, cnoid::VectorX& wa_B, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_B, cnoid::VectorX& dl_B, cnoid::VectorXd& du_B, cnoid::VectorX& wc_B){
      v_ref_ = T_ref_.linear().transpose() * (T_ref_.translation() - prev_T_ref_.translation()) / dt_;//local系
      w_ref_ = cnoid::omegaFromRot(prev_T_ref_.linear().transpose() * T_ref_.linear()) / dt_;//local系

      A_A = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,6);
      for(size_t i=0;i<3;i++)A_A.insert(i,i) = 1.0;
      b_A.resize(3);
      wa_A.resize(3);
      //Cはゼロ
      C_A.resize(0,6);
      dl_A.resize(0);
      du_A.resize(0);
      wc_A.resize(0);

      b_A =
        ((F_.head<3>()-F_ref_.head<3>()).cwiseProduct(force_gain_)
         + (T_.linear().transpose() * (T_ref_.translation() - T_.translation())) * K_p_
         + v_ref_ * D_p_
         + (/*v_ref_ - prev_v_ref_ +*/ prev_v_) * M_p_ / dt_) // ROSを用いる場合はtopicが届く時刻がバラバラなのでrefの加速度は考慮しない
        / (M_p_ / std::pow(dt_,2) + D_p_ / dt_ + K_p_);

      for(size_t i=0;i<3;i++){
        if(b_A[i] > v_limit_ * dt_) b_A[i] = v_limit_ * dt_;
        if(b_A[i] < - v_limit_ * dt_) b_A[i] = - v_limit_ * dt_;
      }

      for(size_t i=0;i<3;i++) wa_A[i] = 1.0;


      A_B = Eigen::SparseMatrix<double,Eigen::RowMajor>(3,6);
      for(size_t i=0;i<3;i++)A_B.insert(i,3+i) = 1.0;
      b_B.resize(3);
      wa_B.resize(3);
      //Cはゼロ
      C_B.resize(0,6);
      dl_B.resize(0);
      du_B.resize(0);
      wc_B.resize(0);

      b_B =
        ((F_.tail<3>()-F_ref_.tail<3>()).cwiseProduct(moment_gain_)
         + cnoid::omegaFromRot(T_.linear().transpose() * T_ref_.linear()) * K_r_
         + w_ref_ * D_r_
         + (/*w_ref_ - prev_w_ref_ +*/ prev_w_) * M_r_ / dt_) // ROSを用いる場合はtopicが届く時刻がバラバラなのでrefの加速度は考慮しない
        / (M_r_ / std::pow(dt_,2) + D_r_ / dt_ + K_r_);

      for(size_t i=0;i<3;i++){
        if(b_B[i] > w_limit_ * dt_) b_B[i] = w_limit_ * dt_;
        if(b_B[i] < - w_limit_ * dt_) b_B[i] = - w_limit_ * dt_;
      }

      for(size_t i=0;i<3;i++) wa_B[i] = 1.0;

      return;
    }

    void Interaction::update(const cnoid::Vector3& v, const cnoid::Vector3& w){//local系
      prev_v_ = v;
      prev_w_ = w;
      prev_v_ref_ = v_ref_;
      prev_w_ref_ = w_ref_;
      prev_T_ref_ = T_ref_;
    }

    std::vector<cnoid::SgNodePtr> Interaction::getDrawOnObjects(){
      cnoidbodyutils::drawCoordsLineCoords(this->lines_,T_, T_ref_);
      return std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    bool loadInteractionFromInfo(const multicontact_controller_msgs::InteractionInfo& info, std::shared_ptr<Interaction>& interaction){
      if(!interaction) interaction = std::make_shared<Interaction>();

      interaction->M_p() = info.M_p;
      interaction->D_p() = info.D_p;
      interaction->K_p() = info.K_p;
      interaction->M_r() = info.M_r;
      interaction->D_r() = info.D_r;
      interaction->K_r() = info.K_r;
      interaction->force_gain() = cnoid::Vector3(info.force_gain.x,info.force_gain.y,info.force_gain.z);
      interaction->moment_gain() = cnoid::Vector3(info.moment_gain.x,info.moment_gain.y,info.moment_gain.z);

      return true;
    }
  }
}
