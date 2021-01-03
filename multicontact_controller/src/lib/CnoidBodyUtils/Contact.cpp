#include <multicontact_controller/lib/CnoidBodyUtils/Contact.h>

#include <cnoid/EigenUtil>
#include <limits>
#include <ros/ros.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    SurfaceContact::SurfaceContact()
      : mu_trans_(0.1),
        max_fz_(200),
        min_fz_(50),
        break_contact_f_v_limit_(25.0)
    {
      type_ = "SURFACE";

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

    bool SurfaceContact::setVertices(const std::vector<cnoid::Vector3f>& vertices){
      if(surface_->polygonVertices().size() != vertices.size()){
        surface_->polygonVertices().resize(vertices.size());
        for(size_t i=0;i<vertices.size(); i++){
          surface_->polygonVertices()[i] = i;
        }
      }

      if(surface_->getOrCreateVertices()->size() != vertices.size()){
        surface_->getOrCreateVertices()->resize(vertices.size());
      }

      for(size_t i=0;i<vertices.size();i++){
        cnoid::Vector3f v = vertices[i];
        v[2] = 0; //Zは無視する
        surface_->getOrCreateVertices()->at(i) = v;
      }

      return true;
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& SurfaceContact::selectMatrix(){
      if(selectMatrix_.cols() != 6 || selectMatrix_.rows() != 6){
        selectMatrix_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6);
        for(size_t i=0;i<6;i++)selectMatrix_.insert(i,i) = 1.0;
      }
      return selectMatrix_;
    }

    // 幾何エラーを返す. このContactPointがどのくらい動けば目標に到達するか.
    // SelectMatrixの次元. m, rad. local系,localまわり.
    cnoid::VectorX SurfaceContact::calcError (const cnoid::Position& current, const cnoid::Position& target){
      cnoid::Vector6 error;
      error.head<3>() = current.linear().transpose() * (target.translation() - current.translation());
      error.tail<3>() = cnoid::omegaFromRot(current.linear().transpose() * target.linear());
      return error;
    }

    //Ax = b, dl <= Cx <= du
    //各行は無次元化されている
    void SurfaceContact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc, bool allow_break_contact){
      // A,bはゼロ．
      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
      b = Eigen::VectorXd(0);
      wa = Eigen::VectorXd(0);

      int dim = 7 + this->surface_->polygonVertices().size();
      C = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,6);
      dl = Eigen::VectorXd::Zero(dim);
      du = Eigen::VectorXd::Zero(dim);
      wc = Eigen::VectorXd::Zero(dim);

      double maximum_distance = this->calcFarthestVertexDistance(this->surface_);

      std::vector<Eigen::Triplet<double> > tripletList;
      tripletList.reserve(dim*2);
      int idx=0;

      //垂直抗力
      {
        double scale = std::max(this->max_fz_,1.0);
        tripletList.push_back(Eigen::Triplet<double>(idx,2,1.0/scale));
        if(allow_break_contact){
          dl[idx] = 0.0;
        }else{
          dl[idx] = std::max(this->min_fz_,0.0) / scale;
        }
        du[idx] = std::max(std::max(this->max_fz_, this->min_fz_) , 0.0) / scale;
        wc[idx] = 1.0;
        idx++;
      }

      //x摩擦
      {
        double scale = std::max(this->max_fz_ * this->mu_trans_, 1.0);
        tripletList.push_back(Eigen::Triplet<double>(idx,0,-1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_/scale));
        dl[idx] = 0.0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
        tripletList.push_back(Eigen::Triplet<double>(idx,0,1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_/scale));
        dl[idx] = 0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
      }

      //y摩擦
      {
        double scale = std::max(this->max_fz_ * this->mu_trans_, 1.0);
        tripletList.push_back(Eigen::Triplet<double>(idx,1,-1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_/scale));
        dl[idx] = 0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
        tripletList.push_back(Eigen::Triplet<double>(idx,1,1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_/scale));
        dl[idx] = 0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
      }

      //回転摩擦
      {
        double scale = std::max(this->max_fz_ * this->mu_trans_ * maximum_distance, 1.0);
        tripletList.push_back(Eigen::Triplet<double>(idx,5,-1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_*maximum_distance/scale));
        dl[idx] = 0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
        tripletList.push_back(Eigen::Triplet<double>(idx,5,1.0/scale));
        tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans_*maximum_distance/scale));
        dl[idx] = 0;
        du[idx] = std::numeric_limits<double>::max();
        wc[idx] = 1.0;
        idx++;
      }

      //COP
      {
        double scale = std::max(this->max_fz_ * maximum_distance, 1.0);
        for(size_t i=0;i<this->surface_->polygonVertices().size();i++){
          cnoid::Vector3f v1 = this->surface_->vertices()->at(this->surface_->polygonVertices()[i]);
          int v2_idx = (i+1 == this->surface_->polygonVertices().size())? 0 : i+1;
          cnoid::Vector3f v2 = this->surface_->vertices()->at(this->surface_->polygonVertices()[v2_idx]);
          tripletList.push_back(Eigen::Triplet<double>(idx,2,(v1[0]*v2[1]-v1[1]*v2[0])/scale));
          tripletList.push_back(Eigen::Triplet<double>(idx,3,(v1[1]-v2[1])/scale));
          tripletList.push_back(Eigen::Triplet<double>(idx,4,(-v1[0]+v2[0])/scale));
          dl[idx] = 0;
          du[idx] = std::numeric_limits<double>::max();
          wc[idx] = 1.0;
          idx++;
        }
      }

      C.setFromTriplets(tripletList.begin(), tripletList.end());
    }

    //Ax = b, dl <= Cx <= du
    //各行は無次元化される
    void SurfaceContact::getBreakContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      // a,bはゼロ
      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
      b = Eigen::VectorXd(0);
      wa = Eigen::VectorXd(0);

      C = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,6);
      dl = Eigen::VectorXd::Zero(1);
      du = Eigen::VectorXd::Zero(1);
      wc = Eigen::VectorXd::Zero(1);

      double scale = std::max(this->max_fz_,1.0);
      C.insert(0,2) = 1.0 / scale;
      du[0] = 0.0;
      dl[0] = -std::numeric_limits<double>::max();
      wc[0] = 1.0;
    }

    //Ax = b, dl <= Cx <= du
    //各行は/iterの次元化される
    // xはfの変化量であり、fの絶対量ではない
    void SurfaceContact::getBreakContactMotionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      // a,bはゼロ
      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
      b = Eigen::VectorXd(0);
      wa = Eigen::VectorXd(0);

      C = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,6);
      dl = Eigen::VectorXd::Zero(1);
      du = Eigen::VectorXd::Zero(1);
      wc = Eigen::VectorXd::Zero(1);

      double scale = std::max(this->max_fz_,1.0);
      C.insert(0,2) = 1.0 / scale;
      if(dt_ > 0.0 && break_contact_f_v_limit_ > 0.0){
        du[0] = - break_contact_f_v_limit_ * dt_ / scale;
      }else{
        std::cerr << "!(dt_ > 0.0 && break_contact_f_v_limit_ = 0.0)" << std::endl;
        du[0] = 0.0;
      }
      dl[0] = -std::numeric_limits<double>::max();
      wc[0] = 1.0;
    }

    //Ax = b, dl <= Cx <= du
    //各行は無次元化される
    void SurfaceContact::getStabilityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6);
      b = Eigen::VectorXd::Zero(6);
      wa = Eigen::VectorXd(6);
      {
        //垂直抗力
        double scale = std::max(this->max_fz_,1.0);
        A.insert(0,0) = 1.0/scale;
        wa[0] = 1.0;
      }
      {
        //x摩擦
        double scale = std::max(this->max_fz_ * this->mu_trans_, 1.0);
        A.insert(1,1) = 1.0/scale;
        wa[1] = 1.0;
        //y摩擦
        A.insert(2,2) = 1.0/scale;
        wa[2] = 1.0;
      }
      double maximum_distance = this->calcFarthestVertexDistance(this->surface_);
      //COP
      {
        double scale = std::max(this->max_fz_ * maximum_distance, 1.0);
        A.insert(3,3) = 1.0/scale;
        wa[3] = 1.0;
        A.insert(4,4) = 1.0/scale;
        wa[4] = 1.0;
      }
      //回転摩擦
      {
        double scale = std::max(this->max_fz_ * this->mu_trans_ * maximum_distance, 1.0);
        A.insert(5,5) = 1.0/scale;
        wa[5] = 1.0;
      }

      // Cはゼロ．
      C = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
      dl = Eigen::VectorXd::Zero(0);
      du = Eigen::VectorXd::Zero(0);
      wc = Eigen::VectorXd::Zero(0);

      return;
    }

    const cnoid::VectorX& SurfaceContact::contactDirection() {
      if(contactDirection_.size() != 6){
        contactDirection_.resize(6);
        contactDirection_[0] = 0;
        contactDirection_[1] = 0;
        contactDirection_[2] = -1;
        contactDirection_[3] = 0;
        contactDirection_[4] = 0;
        contactDirection_[5] = 0;
      }
      return contactDirection_;
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

    double SurfaceContact::calcFarthestVertexDistance(cnoid::SgPolygonMeshPtr surface){
      double maximum = 0.0;
      for(size_t i=0;i<surface->polygonVertices().size();i++){
        double distance = surface->vertices()->at(surface->polygonVertices()[i]).norm();
        if(distance > maximum) maximum = distance;
      }
      return maximum;
    }

    bool loadContactFromInfo(const multicontact_controller_msgs::ContactInfo& info, std::shared_ptr<Contact>& contact){
      if(info.type == "SURFACE"){
        if(!contact || contact->type() != "SURFACE") contact = std::make_shared<SurfaceContact>();
        std::shared_ptr<SurfaceContact> surface_contact = std::dynamic_pointer_cast<SurfaceContact>(contact);
        std::vector<cnoid::Vector3f> vertices;
        for(size_t i=0;i<info.vertices.points.size();i++){
          const geometry_msgs::Point32& point = info.vertices.points[i];
          vertices.push_back(cnoid::Vector3f(point.x, point.y, point.z));
        }
        surface_contact->setVertices(vertices);
        surface_contact->mu_trans() = info.mu_trans;
        surface_contact->max_fz() = info.max_fz;
        surface_contact->min_fz() = info.min_fz;
        surface_contact->contact_v_limit() = info.contact_v_limit;
        surface_contact->break_contact_f_v_limit() = info.break_contact_f_v_limit;
      }else{
        ROS_ERROR("%s is not defined", info.type.c_str());
      }
      return true;
    }
  }
}
