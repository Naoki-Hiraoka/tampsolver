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
    cnoid::Position T_init() const { return T_init_; }
    cnoid::Position& T_init() {return T_init_; }
  private:
    cnoid::Position T_init_;
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
