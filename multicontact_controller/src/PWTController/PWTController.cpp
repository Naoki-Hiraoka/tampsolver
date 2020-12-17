#include <multicontact_controller/PWTController/PWTController.h>

#include <cnoid/EigenUtil>
#include <Eigen/SparseCholesky>
#include <limits>

namespace multicontact_controller {
  SurfaceContact::SurfaceContact()
    : mu_trans_(0.1),
      mu_rot_(0.1),
      max_fz_(200),
      min_fz_(50){
    surface_ = new cnoid::SgPolygonMesh;
    surface_->getOrCreateVertices()->push_back(cnoid::Vector3f(0.1,0.1,0.0));
    surface_->polygonVertices().push_back(0);
    surface_->getOrCreateVertices()->push_back(cnoid::Vector3f(-0.1,0.1,0.0));
    surface_->polygonVertices().push_back(1);
    surface_->getOrCreateVertices()->push_back(cnoid::Vector3f(-0.1,-0.1,0.0));
    surface_->polygonVertices().push_back(2);
    surface_->getOrCreateVertices()->push_back(cnoid::Vector3f(0.1,-0.1,0.0));
    surface_->polygonVertices().push_back(3);
  }

  bool SurfaceContact::setVertices(const std::vector<float>& vertices){
    if(vertices.size() % 3 != 0) return false;

    if(surface_->polygonVertices().size() != vertices.size() / 3){
      surface_->polygonVertices().resize(vertices.size() / 3);
      for(size_t i=0;i<vertices.size() / 3;i++){
        surface_->polygonVertices()[i] = i;
      }
    }

    if(surface_->getOrCreateVertices()->size() != vertices.size() / 3){
      surface_->getOrCreateVertices()->resize(vertices.size() / 3);
    }

    for(size_t i=0;i<vertices.size() / 3;i++){
      surface_->getOrCreateVertices()->at(i) = cnoid::Vector3f(vertices[i*3+0],vertices[i*3+1],0.0);//Zは無視する
    }

    return true;
  }

  //Ax = b, dl <= Cx <= du
  void SurfaceContact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& dl, Eigen::VectorXd& du){
    // A,bはゼロ．
    A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
    b = Eigen::VectorXd(0);

    int dim = 7 + this->surface_->polygonVertices().size();
    C = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,6);
    dl = Eigen::VectorXd::Zero(dim);
    du = Eigen::VectorXd::Zero(dim);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(dim*2);
    int idx=0;

    //垂直抗力
    tripletList.push_back(Eigen::Triplet<double>(idx,2,1));
    dl[idx] = std::max(this->min_fz_,0.0);
    du[idx] = this->max_fz_;
    idx++;

    //x摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,0,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,0,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;

    //y摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,1,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,1,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;

    //回転摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,5,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,5,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot_));
    dl[idx] = 0;
    du[idx] = std::numeric_limits<double>::max();
    idx++;

    //COP
    for(size_t i=0;i<this->surface_->polygonVertices().size();i++){
      cnoid::Vector3f v1 = this->surface_->vertices()->at(this->surface_->polygonVertices()[i]);
      int v2_idx = (i+1 == this->surface_->polygonVertices().size())? 0 : i+1;
      cnoid::Vector3f v2 = this->surface_->vertices()->at(this->surface_->polygonVertices()[v2_idx]);
      tripletList.push_back(Eigen::Triplet<double>(idx,2,v1[0]*v2[1]-v1[1]*v2[0]));
      tripletList.push_back(Eigen::Triplet<double>(idx,3,v1[1]-v2[1]));
      tripletList.push_back(Eigen::Triplet<double>(idx,4,-v1[0]+v2[0]));
      dl[idx] = 0;
      du[idx] = std::numeric_limits<double>::max();
      idx++;
    }

    C.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  std::vector<cnoid::SgNodePtr> SurfaceContact::getDrawOnObjects(const cnoid::Position& T){
    if(!this->lines_){
      this->lines_ = new cnoid::SgLineSet;
      this->lines_->setLineWidth(3.0);
      this->lines_->getOrCreateColors()->resize(1);
      this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,0.5,0.0);
    }

    if(this->lines_->getOrCreateVertices()->size() != this->surface_->vertices()->size())
      this->lines_->getOrCreateVertices()->resize(this->surface_->vertices()->size());
    if(this->lines_->colorIndices().size() != 2*this->surface_->polygonVertices().size()){
      this->lines_->colorIndices().resize(2*this->surface_->polygonVertices().size());
      for(size_t i=0;i<this->lines_->colorIndices().size();i++) this->lines_->colorIndices()[i] = 0;
    }
    if(this->lines_->numLines() != this->surface_->vertices()->size()){
      this->lines_->clearLines();
      for(size_t j=0;j<this->surface_->polygonVertices().size();j++){
        int v1_idx = this->surface_->polygonVertices()[j];
        int v2_idx = this->surface_->polygonVertices()[(j+1 == this->surface_->polygonVertices().size())? 0 : j+1];
        lines_->addLine(v1_idx,v2_idx);
      }
    }

    // update position
    const cnoid::SgVertexArray& vertices = *this->surface_->vertices();
    for(int j=0; j < vertices.size(); j++){
      const cnoid::Vector3 v = T * vertices[j].cast<cnoid::Vector3::Scalar>();
      this->lines_->vertices()->at(j) = v.cast<cnoid::Vector3f::Scalar>();
    }

    return std::vector<cnoid::SgNodePtr>{this->lines_};
  }


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
