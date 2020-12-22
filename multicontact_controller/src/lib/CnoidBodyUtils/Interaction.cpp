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
        K_p_(4000.0),
        M_r_(50.0),
        D_r_(1000.0),
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

    std::vector<cnoid::SgNodePtr> Interaction::getDrawOnObjects(){
      if(!this->lines_){
        lines_ = new cnoid::SgLineSet;
        lines_->setLineWidth(1.0);
        lines_->getOrCreateColors()->resize(4);
        lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,1.0,1.0);
        lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
        lines_->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,1.0,0.0);
        lines_->getOrCreateColors()->at(3) = cnoid::Vector3f(0.0,0.0,1.0);
        // A, A_x, A_y, A_z, B, B_x, B_y, B_z
        lines_->getOrCreateVertices()->resize(8);
        lines_->colorIndices().resize(0);
        lines_->addLine(0,1); lines_->colorIndices().push_back(1); lines_->colorIndices().push_back(1);
        lines_->addLine(0,2); lines_->colorIndices().push_back(2); lines_->colorIndices().push_back(2);
        lines_->addLine(0,3); lines_->colorIndices().push_back(3); lines_->colorIndices().push_back(3);
        lines_->addLine(4,5); lines_->colorIndices().push_back(1); lines_->colorIndices().push_back(1);
        lines_->addLine(4,6); lines_->colorIndices().push_back(2); lines_->colorIndices().push_back(2);
        lines_->addLine(4,7); lines_->colorIndices().push_back(3); lines_->colorIndices().push_back(3);
        lines_->addLine(0,4); lines_->colorIndices().push_back(0); lines_->colorIndices().push_back(0);

      }

      const cnoid::Position& A_pos = T_;
      const cnoid::Position& B_pos = T_ref_;

      lines_->getOrCreateVertices()->at(0) = A_pos.translation().cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(1) = (A_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(2) = (A_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(3) = (A_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(4) = B_pos.translation().cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(5) = (B_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(6) = (B_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
      lines_->getOrCreateVertices()->at(7) = (B_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

      return std::vector<cnoid::SgNodePtr>{this->lines_};;
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
