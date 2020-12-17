#include <multicontact_controller/lib/CnoidBodyUtils/Contact.h>

#include <cnoid/EigenUtil>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    SurfaceContact::SurfaceContact()
      : mu_trans_(0.1),
        mu_rot_(0.1),
        max_fz_(200),
        min_fz_(50){
      type_ = "Surface";

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

    bool loadContactFromInfo(const multicontact_controller_msgs::ContactInfo::ConstPtr& info, std::shared_ptr<Contact>& contact){
      if(info->type == "Surface"){
        if(!contact || contact->type() != "Surface") contact = std::shared_ptr<SurfaceContact>();
        std::shared_ptr<SurfaceContact> surface_contact = std::dynamic_pointer_cast<SurfaceContact>(contact);
        std::vector<cnoid::Vector3f> vertices;
        for(size_t i=0;i<info->vertices.points.size();i++){
          const geometry_msgs::Point32& point = info->vertices.points[i];
          vertices.push_back(cnoid::Vector3f(point.x, point.y, point.z));
        }
        surface_contact->setVertices(vertices);
        surface_contact->mu_trans() = info->mu_trans;
        surface_contact->mu_rot() = info->mu_rot;
        surface_contact->max_fz() = info->max_fz;
        surface_contact->min_fz() = info->min_fz;
      }
      return true;
    }
  }
}
