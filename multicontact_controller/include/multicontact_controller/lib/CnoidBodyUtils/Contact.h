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
      virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc, bool allow_break_contact=false) = 0;
      virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du){
        cnoid::VectorX wa, wc;
        this->getContactConstraint(A,b,wa,C,dl,du,wc);
        return;
      }
      // local座標系. localまわり。次元数はselectmatrixと同じ
      virtual const cnoid::VectorX& contactDirection()=0;

      // local座標系，localまわり Ax = b, dl <= Cx <= du. xの次元数はselectMatrixと同じ
      virtual void getBreakContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) = 0;


      // 拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
      virtual void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc)=0;
      virtual void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du){
        cnoid::VectorX wa, wc;
        this->getStabilityConstraint(A,b,wa,C,dl,du,wc);
        return;
      }


      virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position::Identity()) = 0;

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
      void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc, bool allow_break_contact=false) override;

      void getBreakContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) override;

      void getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc) override;

      const cnoid::VectorX& contactDirection() override;

      std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) override;

      bool setVertices(const std::vector<cnoid::Vector3f>& vertices);// [x1 y1 z1 x2 y2 z2 ...]. Zは無視する
      double mu_trans() const { return mu_trans_;}
      double& mu_trans() { return mu_trans_;}
      double max_fz() const { return max_fz_;}
      double& max_fz() { return max_fz_;}
      double min_fz() const { return min_fz_;}
      double& min_fz() { return min_fz_;}

    protected:
      double calcFarthestVertexDistance(cnoid::SgPolygonMeshPtr surface);
    private:
      cnoid::SgPolygonMeshPtr surface_;//polygonは一つまで．凸形状限定.endeffector相対.
      double mu_trans_;
      double max_fz_;
      double min_fz_;

      cnoid::SgLineSetPtr lines_;//for visualization

      //cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> selectMatrix_; //selectMatrix()
      cnoid::VectorX contactDirection_; //getContactDirection
    };

    bool loadContactFromInfo(const multicontact_controller_msgs::ContactInfo& msg, std::shared_ptr<Contact>& contact);
  };
};

#endif
