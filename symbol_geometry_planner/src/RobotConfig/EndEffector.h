#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <yaml-cpp/yaml.h>

namespace RobotConfig {

  class Contact {

  };

  class SurfaceContact : public Contact {
  public:
    SurfaceContact(cnoid::SgPolygonMeshPtr surface, double mu_trans, double mu_rot, double max_fz, double min_fz);
  private:
    cnoid::SgPolygonMeshPtr surface;
    double mu_trans;
    double mu_rot;
    double max_fz;
    double min_fz;
  };

  class GraspContact : public Contact {
  };

  class EndEffector{
  public:
    EndEffector(const std::string& name, cnoid::Link* link, const cnoid::Position& localpos, std::shared_ptr<Contact> contact);

    std::string& getname() {return name;}
    cnoid::Link* getlink() {return link;}
    cnoid::Position& getlocalpos() {return localpos;}
  private:
    std::string name;
    cnoid::Link* link;
    cnoid::Position localpos;

    std::shared_ptr<Contact> contact;
  };

  void readEndEffectorFromProperties(std::map<std::string, std::shared_ptr<EndEffector> >& endeffectors, cnoid::Body* robot, const YAML::Node& property);

};

#endif
