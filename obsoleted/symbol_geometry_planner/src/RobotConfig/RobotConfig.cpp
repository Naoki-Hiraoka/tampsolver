#include "RobotConfig.h"
#include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <cnoid/BodyLoader>
#include "EditCollisionModel.h"

namespace RobotConfig {
  cnoid::BodyLoader bodyLoader;

  RobotConfig::RobotConfig (const std::string& name, const std::string& url, std::ostream& os)
  {
    this->loadConfFile(name,url,os);
    this->setupModel();
  }

  bool RobotConfig::loadConfFile(const std::string& name, const std::string& url, std::ostream& os){
    YAML::Node config = YAML::LoadFile(url);

    if(config["model"].IsDefined()) {
      std::string filename = config["model"].as<std::string>();
      std::string packagestr = "package://";
      if(filename.size()>packagestr.size() && filename.substr(0,packagestr.size()) == packagestr){
        filename = filename.substr(packagestr.size());
        int pos = filename.find("/");
        filename = ros::package::getPath(filename.substr(0,pos)) + filename.substr(pos);
      }

      bodyLoader.setMessageSink(os);
      this->robot = bodyLoader.load(filename);
      if(!this->robot){
        std::cerr << "[RobotConfig] load " << config["model"] << " failed" << std::endl;
        return false;
      }
      if(name!="") this->robot->setName(name);
    } else {
      std::cerr << "[RobotConfig] model is not defined" << std::endl;
      return false;
    }

    if(config["collision_model"].IsDefined()) {
      if(config["collision_model"].as<std::string>() == "convex hull"){
        for(size_t i=0;i<this->robot->numLinks();i++){
          cnoid::SgNode* coldetModel = convertToConvexHull(this->robot->link(i)->collisionShape());
          if(coldetModel){
            this->robot->link(i)->setCollisionShape(coldetModel);
          }else{
            std::cerr << "[RobotConfig] convex hull " << this->robot->link(i)->name() << " fail" << std::endl;
          }
        }
      }else{
        std::cerr << "[RobotConfig] collision_model " << config["collision_model"] << " is not defined" << std::endl;
      }
    }

    if(config["end_effectors"].IsDefined()) {
      readEndEffectorFromProperties(this->endeffectors,
                                        this->robot,
                                        config["end_effectors"]);
    }

    if(config["collision_pair"].IsDefined()) {
      std::stringstream ss(config["collision_pair"].as<std::string>());
      std::string item;
      while (std::getline(ss, item, ' ')) {
        std::stringstream _ss(item);
        std::vector<std::string> _item(2);
        std::getline(_ss, _item[0], ':');
        std::getline(_ss, _item[1], ':');
        cnoid::Link* link0 = robot->link(_item[0]);
        cnoid::Link* link1 = robot->link(_item[1]);
        if ( !link0 || !link1) {
          std::cerr << "[RobotConfig::RobotConfig] collision_pair" << _item[0] << ":" << _item[1] << " fail" << std::endl;
          continue;
        }
        this->collision_pairs.push_back(std::pair<cnoid::Link*,cnoid::Link*>(link0,link1));
      }
    }

    if(config["joint_limit"].IsDefined()) {
      const YAML::Node& property = config["joint_limit"];
      for(size_t i=0;i<property.size();i++){
        if(!property[i]["joint"].IsDefined()){
          std::cerr << "[loadConfFile] joint_limit name is not defined for " << i << std::endl;
          continue;
        }
        std::string name = property[i]["joint"].as<std::string>();
        cnoid::Link* link=robot->link(name);
        if(!link){
          std::cerr << "[loadConfFile] joint_limit link " << name <<  " is not found" << std::endl;
          continue;
        }
        double llimit = -1e30;
        double ulimit = 1e30;
        if(property[i]["llimit"].IsDefined()) llimit = property[i]["llimit"].as<double>();
        if(property[i]["ulimit"].IsDefined()) ulimit = property[i]["ulimit"].as<double>();
        this->joint_limits[link] = std::pair<double,double>(llimit,ulimit);
      }
    }

    if(config["joint_limit_table"].IsDefined()) {
      readJointLimitTableFromProperties(this->joint_mm_tables,
                                        this->robot,
                                        config["joint_limit_table"].as<std::string>());
    }

    // if(config["interfaces"].IsDefined()) {
    //   readInterfaceFromProperties(this->interfaces,
    //                               this->robot,
    //                               config["interfaces"].as<std::string>());
    // }

    return true;
  }

  bool RobotConfig::setupModel(){
    for(size_t i=0;i<this->robot->numLinks();i++){
      std::shared_ptr<Vclip::Polyhedron> vcliplink(convertToVClipModel(this->robot->link(i)->collisionShape()));
      this->vcliplinks[this->robot->link(i)] = vcliplink;
    }
  }
};
