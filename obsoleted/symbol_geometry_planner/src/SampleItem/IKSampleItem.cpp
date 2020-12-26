#include "IKSampleItem.h"
#include "../RobotConfig/RobotConfig.h"
#include "../IK/IKsolver.h"
#include "../IK/Constraints/PositionConstraint.h"
#include "../IK/Constraints/CollisionConstraint.h"
#include "../IK/Constraints/CddSCFRConstraint.h"
#include "../IK/Constraints/CddSCFRConstraint2.h"
#include "../IK/Constraints/LPSCFRConstraint.h"
#include "../IK/Util.h"
#include <cnoid/TimeMeasure>
#include <ros/package.h>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

  void IKSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<IKSampleItem>("IKSampleItem");
  }


  IKSampleItem::IKSampleItem()
    : PlannerBaseItem()
  {
  }


  IKSampleItem::IKSampleItem(const IKSampleItem& org)
    : PlannerBaseItem(org)
  {
  }



  IKSampleItem::~IKSampleItem()
  {
  }


  void IKSampleItem::main()
  {
    // load config
    std::string filepath = ros::package::getPath("symbol_geometry_planner") + "/sample/HRP2JSKNTSconfig.yaml";
    std::shared_ptr<RobotConfig::RobotConfig> config = std::make_shared<RobotConfig::RobotConfig>("HRP2JSKNTS",filepath);
    BodyItemPtr robot = this->instantiate(config->get_robot());
    this->objects(std::set<BodyItemPtr>{robot});

    // reset manip pose
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0,
        0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.698132,
        0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 0.0,
        0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for(int j=0; j < robot->body()->numJoints(); ++j){
      robot->body()->joint(j)->q() = reset_manip_pose[j];
    }
    robot->body()->calcForwardKinematics();
    robot->body()->calcCenterOfMass();
    this->drawObjects();
    cnoid::msleep(500);

    std::cerr << "reset-manip-pose" << std::endl;

    // set goal
    std::vector<cnoid::Body*> variables{robot->body()};
    std::vector<std::shared_ptr<IK::IKConstraint> > tasks;
    std::vector<std::shared_ptr<IK::IKConstraint> > constraints;

    // task: rarm to target
    {
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(0.2,0.3,0.3);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5,cnoid::Vector3(0,1,0)));
      //pos.linear() = (eef_links[2]->T()*eef_localposs[2]).linear();
      tasks.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["rhand"],
                                                               pos));
    }

    // constraint: joint mim-max
    {
      std::vector<std::shared_ptr<IK::IKConstraint> > minmaxconstraints = IK::generateMinMaxConstraints(robot->body(),config);
      std::copy(minmaxconstraints.begin(),minmaxconstraints.end(),std::back_inserter(constraints));
    }

    // constraint: self-collision
    {
      std::vector<std::shared_ptr<IK::IKConstraint> > collisionconstraints = IK::generateCollisionConstraints(robot->body(),config,"VClip");
      std::copy(collisionconstraints.begin(),collisionconstraints.end(),std::back_inserter(constraints));
    }

    // constraint: rleg & lleg not move
    constraints.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["rleg"],
                                                                   config->get_endeffectors()["rleg"]->getlink()->T()*config->get_endeffectors()["rleg"]->getlocalpos()));
    constraints.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["lleg"],
                                                                   config->get_endeffectors()["lleg"]->getlink()->T()*config->get_endeffectors()["lleg"]->getlocalpos()));

    // constraint: support body by rleg & lleg
    {
      //constraints.push_back(std::make_shared<IK::CddSCFRConstraint>(robot->body(),
      //constraints.push_back(std::make_shared<IK::CddSCFRConstraint2>(robot->body(),
      constraints.push_back(std::make_shared<IK::LPSCFRConstraint>(robot->body(),
                                                                     std::vector<std::shared_ptr<RobotConfig::EndEffector> >{config->get_endeffectors()["rleg"],config->get_endeffectors()["lleg"]}));
    }

    // solve ik
    IK::IKsolver solver(variables,tasks,constraints);
    int debuglevel = 0;
    solver.set_debug_level(debuglevel);
    for(size_t i=0;i<tasks.size();i++) tasks[i]->set_debug_level(debuglevel);
    for(size_t i=0;i<constraints.size();i++) constraints[i]->set_debug_level(debuglevel);

    cnoid::TimeMeasure timer;
    for(size_t i=0; i<100;i++){
      timer.begin();

      bool solved = solver.solve_one_loop();

      double time1 = timer.measure();
      timer.begin();

      this->drawObjects(false);
      std::vector<cnoid::SgNodePtr> drawonobjects = solver.getDrawOnObjects();
      for(size_t j=0;j<drawonobjects.size();j++){
        this->drawOn(drawonobjects[j]);
      }
      this->flush();

      double time2 = timer.measure();

      this->os << i
               << " solve time: " << time1
               << " draw time: " << time2
               << std::endl;

      if(solved)break;

      //cnoid::msleep(50); //sleepがあると，sleepしていない部分の処理まで遅くなるみたい?
    }

  }
}

