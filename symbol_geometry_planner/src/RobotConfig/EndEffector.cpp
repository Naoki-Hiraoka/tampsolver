#include "EndEffector.h"
#include <iostream>

namespace RobotConfig{
  SurfaceContact::SurfaceContact(cnoid::SgPolygonMeshPtr _surface, double _mu_trans, double _mu_rot, double _max_fz, double _min_fz):
    surface(_surface),
    mu_trans(_mu_trans),
    mu_rot(_mu_rot),
    max_fz(_max_fz),
    min_fz(_min_fz)
  {
  }

  EndEffector::EndEffector(const std::string& _name, cnoid::Link* _link, const cnoid::Position& _localpos, std::shared_ptr<Contact> _contact):
    name(_name),
    link(_link),
    localpos(_localpos),
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
        localpos.linear() = cnoid::Matrix3(cnoid::AngleAxis(angle,axis));
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

      endeffectors[name]=std::make_shared<EndEffector>(name,link,localpos,contact);
    }
  }
};
