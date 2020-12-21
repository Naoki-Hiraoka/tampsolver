#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_CONTACT_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_CONTACT_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <cnoid/SceneDrawables>
#include <Eigen/Sparse>
#include <multicontact_controller_msgs/ContactInfo.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    class Contact {
    public:
      // ? x 6. 力や幾何制約の存在する成分を抽出
      virtual const Eigen::SparseMatrix<double,Eigen::RowMajor>& selectMatrix()=0;
      // local座標系，localまわり Ax = b, dl <= Cx <= du. xの次元数はselectMatrixと同じ
      virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) = 0;
      virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du){
        cnoid::VectorX wa, wc;
        this->getContactConstraint(A,b,wa,C,dl,du,wc);
        return;
      }

      // 拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
      virtual void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc)=0;
      virtual void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du){
        cnoid::VectorX wa, wc;
        this->getStabilityConstraint(A,b,wa,C,dl,du,wc);
        return;
      }


      virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) = 0;

      std::string type() const { return type_;}
      std::string& type() { return type_;}
    protected:
      std::string type_;
    };

    class SurfaceContact : public Contact {
    public:
      SurfaceContact();
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& selectMatrix() override;
      // local座標系，localまわり Ax = b, dl <= Cx <= du
      void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) override;

      void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) override;

      std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) override;

      bool setVertices(const std::vector<cnoid::Vector3f>& vertices);// [x1 y1 z1 x2 y2 z2 ...]. Zは無視する
      double mu_trans() const { return mu_trans_;}
      double& mu_trans() { return mu_trans_;}
      double mu_rot() const { return mu_rot_;}
      double& mu_rot() { return mu_rot_;}
      double max_fz() const { return max_fz_;}
      double& max_fz() { return max_fz_;}
      double min_fz() const { return min_fz_;}
      double& min_fz() { return min_fz_;}

      double& weight_fz_constraint() { return weight_fz_constraint_;}
      double weight_fz_constraint() const { return weight_fz_constraint_;}
      double& weight_fxy_constraint() { return weight_fxy_constraint_;}
      double weight_fxy_constraint() const { return weight_fxy_constraint_;}
      double& weight_nz_constraint() { return weight_nz_constraint_;}
      double weight_nz_constraint() const { return weight_nz_constraint_;}
      double& weight_nxy_constraint() { return weight_nxy_constraint_;}
      double weight_nxy_constraint() const { return weight_nxy_constraint_;}
      double& weight_fz_reduction() { return weight_fz_reduction_;}
      double weight_fz_reduction() const { return weight_fz_reduction_;}
      double& weight_fxy_reduction() { return weight_fxy_reduction_;}
      double weight_fxy_reduction() const { return weight_fxy_reduction_;}
      double& weight_nz_reduction() { return weight_nz_reduction_;}
      double weight_nz_reduction() const { return weight_nz_reduction_;}
      double& weight_nxy_reduction() { return weight_nxy_reduction_;}
      double weight_nxy_reduction() const { return weight_nxy_reduction_;}
    private:
      cnoid::SgPolygonMeshPtr surface_;//polygonは一つまで．凸形状限定.endeffector相対.
      double mu_trans_;
      double mu_rot_;//自動で算出できるはずTODO
      double max_fz_;
      double min_fz_;

      // for optimization //自動で算出できるはずTODO
      double weight_fz_constraint_;
      double weight_fxy_constraint_;
      double weight_nxy_constraint_;
      double weight_nz_constraint_;
      double weight_fz_reduction_;
      double weight_fxy_reduction_;
      double weight_nxy_reduction_;
      double weight_nz_reduction_;

      cnoid::SgLineSetPtr lines_;//for visualization

      //cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> selectMatrix_; //selectMatrix()
    };

    bool loadContactFromInfo(const multicontact_controller_msgs::ContactInfo::ConstPtr& msg, std::shared_ptr<Contact>& contact);
  };
};

#endif
