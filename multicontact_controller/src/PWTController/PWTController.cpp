#include <multicontact_controller/PWTController/PWTController.h>

#include <cnoid/EigenUtil>
#include <Eigen/SparseCholesky>
#include <limits>

namespace multicontact_controller {


  bool PWTController::calcRootOdometry(cnoid::Body* robot, std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints){
    if(error_.size() != contactPoints.size()*4) error_.resize(contactPoints.size()*4);
    if(J_.rows() != contactPoints.size()*4 || J_.cols() != 4) J_.conservativeResize(contactPoints.size()*4,4);
    if(W_.rows() != contactPoints.size()*4 || W_.cols() != contactPoints.size()*4) W_.conservativeResize(contactPoints.size()*4,contactPoints.size()*4);
    for(size_t loop=0;loop<loopNum_;loop++){
      cnoid::Position originT = this->getOriginCoords();

      // calc error, jacobian
      for(size_t i=0;i<contactPoints.size();i++){
        const cnoid::Position T_ref;
        const cnoid::Position T_cur = originT * contactPoints[i]->parent()->T() * contactPoints[i]->T_local();

        error_.segment<3>(4*i) = T_ref.translation() - T_cur.translation();
        cnoid::Matrix3 diffR = T_ref.linear() * T_cur.linear().inverse();
        cnoid::Matrix3 horizontal_diffR = cnoidbodyutils::orientCoordsToAxis(diffR);
        error_[4*i+3] = cnoid::rpyFromRot(horizontal_diffR)[2];

        cnoid::Vector3 dp = cnoid::Vector3::UnitZ().cross(T_cur.translation() - robot->rootLink()->p());
        J_.coeffRef(4*i+0,0) = 1.0; J_.coeffRef(4*i+0,3) = dp[0];
        J_.coeffRef(4*i+1,1) = 1.0; J_.coeffRef(4*i+1,3) = dp[1];
        J_.coeffRef(4*i+2,2) = 1.0; J_.coeffRef(4*i+2,3) = dp[2];
        J_.coeffRef(4*i+3,3) = 1.0;
      }

      // solve
      for(size_t i=0;i<W_.cols();i++){
        if(i%4==3) W_.coeffRef(i,i) = 0.01;//yawの寄与を小さく
        else W_.coeffRef(i,i) = 1.0;
      }
      // d_origin = (Jt W J)^-1 Jt W error
      // <=>
      // (Jt W J) d_origin = Jt W error
      solver_.compute(J_.transpose() * W_ * J_);
      if(solver_.info()!=Eigen::Success) {
        // decomposition failed
        return false;
      }
      cnoid::Vector4 d_origin = solver_.solve(J_.transpose() * W_ * error_);
      if(solver_.info()!=Eigen::Success) {
        // solving failed
        return false;
      }

      // apply result
      originp_ += d_origin.head<3>();
      originyaw_ += d_origin[3];
    }

    return true;
  }

  bool PWTController::setRootOdom(cnoid::Body* robot, const cnoid::Position& odomT){
    if(!robot) return false;

    cnoid::Position originCoords = odomT * robot->rootLink()->T().inverse();
    originp_ = originCoords.translation();
    cnoid::Matrix3 horizontalR = cnoidbodyutils::orientCoordsToAxis(originCoords.linear());
    originyaw_ = cnoid::rpyFromRot(horizontalR)[2];

    return true;
  }

  cnoid::Position PWTController::getOriginCoords(){
    cnoid::Position originCoords;
    originCoords.translation() = originp_;
    originCoords.linear() = cnoid::rotFromRpy(0,0,originyaw_);
    return originCoords;
  }
};
