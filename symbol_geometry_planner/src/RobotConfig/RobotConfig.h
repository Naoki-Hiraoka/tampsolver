#ifndef ROBOTCONFIG_H
#define ROBOTCONFIG_H

#include <cnoid/Body>
#include <memory>
#include <vclip.h>
#include "JointLimitTable.h"
#include "EndEffector.h"

namespace RobotConfig {
  class RobotConfig {
  public:
    RobotConfig (const std::string& name, const std::string& url, std::ostream& os=std::cerr);
    cnoid::Body* get_robot() {return robot;}
    std::map<std::string, std::shared_ptr<EndEffector> >& get_endeffectors() {return endeffectors;}
    std::vector<std::pair<cnoid::Link*, cnoid::Link*> >& get_collision_pairs() {return collision_pairs;}
    std::map<const cnoid::Link*, std::pair<double,double> >& get_joint_limits() {return joint_limits;}
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> >& get_joint_mm_tables() {return joint_mm_tables;}
    std::map<const cnoid::Link*, std::shared_ptr<Vclip::Polyhedron> >& get_vcliplinks() {return vcliplinks;}
  private:
    bool loadConfFile(const std::string& name, const std::string& url, std::ostream& os=std::cerr);
    bool setupModel();

    cnoid::Body* robot;
    std::map<std::string, std::shared_ptr<EndEffector> > endeffectors;
    std::vector<std::pair<cnoid::Link*, cnoid::Link*> > collision_pairs;
    std::map<const cnoid::Link*, std::pair<double,double> > joint_limits;
    std::map<const cnoid::Link*,std::shared_ptr<JointLimitTable> > joint_mm_tables;

    std::map<const cnoid::Link*, std::shared_ptr<Vclip::Polyhedron> > vcliplinks;
  };
};

#endif
