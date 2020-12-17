#ifndef PWT_CONTROLLER_H
#define PWT_CONTROLLER_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <memory>
#include <Eigen/Sparse>
#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {
  class Contact {
  public:
    // local座標系，localまわり Ax = b, dl <= Cx <= du
    virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du) = 0;
    virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) = 0;
  protected:

  };

  class SurfaceContact : public Contact {
  public:
    SurfaceContact();
    SurfaceContact(cnoid::SgPolygonMeshPtr surface, double mu_trans, double mu_rot, double max_fz, double min_fz);
    // local座標系，localまわり Ax = b, dl <= Cx <= du
    void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du) override;

    std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) override;

    bool setVertices(const std::vector<float>& vertices);// [x1 y1 z1 x2 y2 z2 ...]. Zは無視する
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


  class ContactPointPWTC: public cnoidbodyutils::ContactPoint {
  public:
    // odom系の座標のref値
    cnoid::Position T_ref() const { return T_ref_; }
    cnoid::Position& T_ref() {return T_ref_; }
    cnoid::Position prev_T_ref() const { return prev_T_ref_; }
    cnoid::Position& prev_T_ref() {return prev_T_ref_; }
    cnoid::Position prev_prev_T_ref() const { return prev_prev_T_ref_; }
    cnoid::Position& prev_prev_T_ref() {return prev_prev_T_ref_; }
  private:
    cnoid::Position T_ref_;
    cnoid::Position prev_T_ref_;
    cnoid::Position prev_prev_T_ref_;

    std::string type_;
  };

  class PWTController {
  public:
    PWTController()
      : originp_(cnoid::Vector3::Zero()),
        originyaw_(0),
        loopNum_(3){
    }

    // 各ContactPointの座標がT_initと一致するためにrobotのrootに左から掛けるべきTを,前回のoriginp, originyawをTの初期値として探索する
    bool calcRootOdometry(cnoid::Body* robot, std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints);

    // robotのrootがodomTと一致するためのrobotのrootに左から掛けるべきTを計算しrootp, rootyawに格納する
    bool setRootOdom(cnoid::Body* robot, const cnoid::Position& odomT);

    // robotのrootに左から掛けるべきT
    cnoid::Position getOriginCoords();

    size_t loopNum() const {return loopNum_;}
    size_t& loopNum() {return loopNum_;}
  private:
    cnoid::Vector3 originp_;
    double originyaw_;

    size_t loopNum_;

    // cache
    Eigen::SimplicialLDLT<Eigen::SparseMatrix<double> > solver_;
    cnoid::VectorX error_ /*x,y,z,yaw,...*/;
    Eigen::SparseMatrix<double> J_;
    Eigen::SparseMatrix<double> W_;
  };

};

#endif
