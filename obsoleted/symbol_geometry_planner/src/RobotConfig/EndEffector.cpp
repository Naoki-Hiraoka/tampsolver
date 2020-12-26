#include "EndEffector.h"
#include <iostream>

namespace RobotConfig{
  void Contact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& lb, Eigen::VectorXd& ub){
    C = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
    lb = Eigen::VectorXd(0);
    ub = Eigen::VectorXd(0);
  }

  void Contact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d){
    A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
    b = Eigen::VectorXd(0);
    C = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
    d = Eigen::VectorXd(0);
  }

  std::vector<cnoid::SgNodePtr> Contact::getDrawOnObjects(const cnoid::Position& T){
    return std::vector<cnoid::SgNodePtr>();
  }

  SurfaceContact::SurfaceContact(cnoid::SgPolygonMeshPtr _surface, double _mu_trans, double _mu_rot, double _max_fz, double _min_fz):
    surface(_surface),
    mu_trans(_mu_trans),
    mu_rot(_mu_rot),
    max_fz(_max_fz),
    min_fz(_min_fz)
  {
  }

  void SurfaceContact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& lb, Eigen::VectorXd& ub){
    int dim = 7 + this->surface->polygonVertices().size();
    C = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,6);
    lb = Eigen::VectorXd::Zero(dim);
    ub = Eigen::VectorXd::Zero(dim);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(dim*2);
    int idx=0;

    //垂直抗力
    tripletList.push_back(Eigen::Triplet<double>(idx,2,1));
    lb[idx] = std::max(this->min_fz,0.0);
    ub[idx] = this->max_fz;
    idx++;

    //x摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,0,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,0,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;

    //y摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,1,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,1,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;

    //回転摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,5,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,5,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot));
    lb[idx] = 0;
    ub[idx] = 1e30;
    idx++;

    //COP
    for(size_t i=0;i<this->surface->polygonVertices().size();i++){
      cnoid::Vector3f v1 = this->surface->vertices()->at(this->surface->polygonVertices()[i]);
      int v2_idx = (i+1 == this->surface->polygonVertices().size())? 0 : i+1;
      cnoid::Vector3f v2 = this->surface->vertices()->at(this->surface->polygonVertices()[v2_idx]);
      tripletList.push_back(Eigen::Triplet<double>(idx,2,v1[0]*v2[1]-v1[1]*v2[0]));
      tripletList.push_back(Eigen::Triplet<double>(idx,3,v1[1]-v2[1]));
      tripletList.push_back(Eigen::Triplet<double>(idx,4,-v1[0]+v2[0]));
      lb[idx] = 0;
      ub[idx] = 1e30;
      idx++;
    }

    C.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  //Ax = b, Cx >= d
  void SurfaceContact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d){
    // A,bはゼロ．垂直抗力のfz成分のmaxが，行を分けるだけで後はSurfaceContact::getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& lb, Eigen::VectorXd& ub)と同じ
    A = Eigen::SparseMatrix<double,Eigen::RowMajor>(0,6);
    b = Eigen::VectorXd(0);

    int dim = 8 + this->surface->polygonVertices().size();
    C = Eigen::SparseMatrix<double,Eigen::RowMajor>(dim,6);
    d = Eigen::VectorXd::Zero(dim);

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(dim*2);
    int idx=0;

    //垂直抗力
    tripletList.push_back(Eigen::Triplet<double>(idx,2,1));
    d[idx] = std::max(this->min_fz,0.0);
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,2,-1));
    d[idx] = -this->max_fz;
    idx++;

    //x摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,0,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    d[idx] = 0;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,0,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    d[idx] = 0;
    idx++;

    //y摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,1,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    d[idx] = 0;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,1,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_trans));
    d[idx] = 0;
    idx++;

    //回転摩擦
    tripletList.push_back(Eigen::Triplet<double>(idx,5,-1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot));
    d[idx] = 0;
    idx++;
    tripletList.push_back(Eigen::Triplet<double>(idx,5,1));
    tripletList.push_back(Eigen::Triplet<double>(idx,2,this->mu_rot));
    d[idx] = 0;
    idx++;

    //COP
    for(size_t i=0;i<this->surface->polygonVertices().size();i++){
      cnoid::Vector3f v1 = this->surface->vertices()->at(this->surface->polygonVertices()[i]);
      int v2_idx = (i+1 == this->surface->polygonVertices().size())? 0 : i+1;
      cnoid::Vector3f v2 = this->surface->vertices()->at(this->surface->polygonVertices()[v2_idx]);
      tripletList.push_back(Eigen::Triplet<double>(idx,2,v1[0]*v2[1]-v1[1]*v2[0]));
      tripletList.push_back(Eigen::Triplet<double>(idx,3,v1[1]-v2[1]));
      tripletList.push_back(Eigen::Triplet<double>(idx,4,-v1[0]+v2[0]));
      d[idx] = 0;
      idx++;
    }

    C.setFromTriplets(tripletList.begin(), tripletList.end());
  }

  std::vector<cnoid::SgNodePtr> SurfaceContact::getDrawOnObjects(const cnoid::Position& T){
    if(!this->lines){
      lines = new cnoid::SgLineSet;
      lines->setLineWidth(3.0);
      lines->getOrCreateColors()->resize(1);
      lines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,0.5,0.0);
      lines->getOrCreateVertices()->resize(this->surface->vertices()->size());
      lines->colorIndices().resize(0);
      for(size_t j=0;j<this->surface->polygonVertices().size();j++){
        int v1_idx = this->surface->polygonVertices()[j];
        int v2_idx = this->surface->polygonVertices()[(j+1 == this->surface->polygonVertices().size())? 0 : j+1];
        lines->addLine(v1_idx,v2_idx); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
      }
    }

    // update position
    const cnoid::SgVertexArray& vertices = *this->surface->vertices();
    for(int j=0; j < vertices.size(); j++){
      const cnoid::Vector3 v = T * vertices[j].cast<cnoid::Vector3::Scalar>();
      lines->vertices()->at(j) = v.cast<cnoid::Vector3f::Scalar>();
    }

    return std::vector<cnoid::SgNodePtr>{lines};
  }


  EndEffector::EndEffector(const std::string& _name, cnoid::Link* _link, const cnoid::Position& _localpos, const std::string &_identifier, std::shared_ptr<Contact> _contact):
    name(_name),
    link(_link),
    localpos(_localpos),
    identifier(_identifier),
    contact(_contact)
  {
  }

  void readEndEffectorFromProperties(std::map<std::string, std::shared_ptr<EndEffector> >& endeffectors, cnoid::Body* robot, const YAML::Node& property){
    for(size_t i=0;i<property.size();i++){
      if(!property[i]["name"].IsDefined()){
        std::cerr << "[readEndEffectorFromProperties] name is not defined for " << i << std::endl;
        continue;
      }
      std::string name=property[i]["name"].as<std::string>();

      if(!property[i]["link"].IsDefined()){
        std::cerr << "[readEndEffectorFromProperties] link is not defined for " << name << std::endl;
        continue;
      }
      std::string linkname=property[i]["link"].as<std::string>();
      cnoid::Link* link=robot->link(linkname);
      if(!link){
        std::cerr << "[readEndEffectorFromProperties] link " << linkname <<  " is not found for " << name << std::endl;
        continue;
      }

      cnoid::Position localpos;
      if(!property[i]["localpos"].IsDefined()){
        localpos.translation() = cnoid::Vector3::Zero();
        localpos.linear() = cnoid::Matrix3::Identity();
      }else{
        std::stringstream ss(property[i]["localpos"].as<std::string>());
        std::string item;
        cnoid::Vector3 pos;
        for(size_t j=0;j<3;j++){
          std::getline(ss, item, ',');
          pos[j]=std::stod(item);
        }
        cnoid::Vector3 axis;
        for(size_t j=0;j<3;j++){
          std::getline(ss, item, ',');
          axis[j]=std::stod(item);
        }
        std::getline(ss, item, ',');
        double angle = std::stod(item);
        localpos.translation() = pos;
        localpos.linear() = cnoid::Matrix3(cnoid::AngleAxis(angle,axis.normalized()));
      }

      std::string identifier;
      if(property[i]["identifier"].IsDefined()){
        identifier=property[i]["identifier"].as<std::string>();
      }

      std::shared_ptr<Contact> contact;
      if(!property[i]["contact"].IsDefined() || !property[i]["contact"]["type"].IsDefined()){
        contact = std::make_shared<Contact>();
      }else{
        const YAML::Node& contact_property = property[i]["contact"];
        std::string type = contact_property["type"].as<std::string>();
        if(type == "Surface"){
          if(!contact_property["vertices"].IsDefined()){
            std::cerr << "[readEndEffectorFromProperties] vertices is not defined for " << name << std::endl;
            continue;
          }
          cnoid::SgPolygonMeshPtr surface = new cnoid::SgPolygonMesh;
          std::vector<float> vertices;
          std::stringstream ss(contact_property["vertices"].as<std::string>());
          std::string item;
          while(std::getline(ss, item, ',')){
            vertices.push_back(std::stof(item));
          }
          for(size_t j=0;j<vertices.size()/2;j++){
            surface->getOrCreateVertices()->push_back(cnoid::Vector3f(vertices[j*2+0],vertices[j*2+1],0.0));
            surface->polygonVertices().push_back(j);
          }

          double mu_trans = (contact_property["mu_trans"].IsDefined())? contact_property["mu_trans"].as<double>() : 0.1;
          double mu_rot = (contact_property["mu_rot"].IsDefined())? contact_property["mu_rot"].as<double>() : 0.01;
          double max_fz = (contact_property["max_fz"].IsDefined())? contact_property["max_fz"].as<double>() : 500;
          double min_fz = (contact_property["mu_trans"].IsDefined())? contact_property["min_fz"].as<double>() : 50;

          contact = std::make_shared<SurfaceContact>(surface, mu_trans, mu_rot, max_fz, min_fz);
        }else{
          contact = std::make_shared<Contact>();
        }
      }

      endeffectors[name]=std::make_shared<EndEffector>(name,link,localpos,identifier,contact);
    }
  }
};
