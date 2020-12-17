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
      // local座標系，localまわり Ax = b, dl <= Cx <= du
      virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du) = 0;
      virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) = 0;

      std::string type() const { return type_;}
      std::string& type() { return type_;}
    protected:
      std::string type_;
    };

    class SurfaceContact : public Contact {
    public:
      SurfaceContact();
      // local座標系，localまわり Ax = b, dl <= Cx <= du
      void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du) override;

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
    private:
      cnoid::SgPolygonMeshPtr surface_;//polygonは一つまで．凸形状限定.endeffector相対.
      double mu_trans_;
      double mu_rot_;
      double max_fz_;
      double min_fz_;

      cnoid::SgLineSetPtr lines_;//for visualization
    };

    bool loadContactFromInfo(const multicontact_controller_msgs::ContactInfo::ConstPtr& msg, std::shared_ptr<Contact>& contact);
  };
};

#endif
