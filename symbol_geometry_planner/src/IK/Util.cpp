#include "Util.h"
#include "Constraints/AISTCollisionConstraint.h"
#include "Constraints/VClipCollisionConstraint.h"

namespace IK{
  //全linkpairの干渉回避制約を作る
  std::vector<std::shared_ptr<IKConstraint> > generateCollisionConstraints(cnoid::Body* robot, std::shared_ptr<RobotConfig::RobotConfig> config, const std::string& mode){
    std::vector<std::shared_ptr<IKConstraint> > constraints;
    for(size_t i=0;i<config->get_collision_pairs().size();i++){
      if (config->get_collision_pairs()[i].first->body() == robot && config->get_collision_pairs()[i].second->body() == robot){
        if(mode == "AIST"){
          constraints.push_back(std::make_shared<AISTCollisionConstraint>(config->get_collision_pairs()[i].first,config->get_collision_pairs()[i].second));
        }else if(mode == "VClip"){
          constraints.push_back(std::make_shared<VClipCollisionConstraint>(config->get_collision_pairs()[i].first,config->get_collision_pairs()[i].second,config->get_vcliplinks()[config->get_collision_pairs()[i].first].get(),config->get_vcliplinks()[config->get_collision_pairs()[i].second].get()));
        }else{
          std::cerr << "[generateCollisionConstraints] mode " << mode << " not defined" << std::endl;
        }
      }
    }
    return constraints;
  }


  std::vector<std::shared_ptr<IKConstraint> > generateMinMaxConstraints(cnoid::Body* robot, std::shared_ptr<RobotConfig::RobotConfig> config){
    std::vector<std::shared_ptr<IKConstraint> > constraints;
    for(size_t j=0;j<robot->numJoints();j++){
      if(config->get_joint_mm_tables().find(robot->joint(j)) != config->get_joint_mm_tables().end())
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->joint(j), config->get_joint_mm_tables()[robot->joint(j)]));
      else
        constraints.push_back(std::make_shared<MinMaxJointConstraint>(robot->joint(j), nullptr));
    }
    return constraints;
  }
}
