#include "ReachSampleItem.h"
#include "../RobotConfig/RobotConfig.h"
#include "../Utility/Util.h"
#include "../IK/IKsolver.h"
#include "../IK/Constraints/PositionConstraint.h"
#include "../IK/Constraints/CollisionConstraint.h"
#include "../IK/Constraints/LPSCFRConstraint.h"
#include "../IK/Util.h"
#include <ros/package.h>
#include <cnoid/TimeMeasure>

using namespace cnoid;

namespace cnoid {

  void ReachSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<ReachSampleItem>("ReachSampleItem");
  }


  void ReachSampleItem::main()
  {
    // load config
    std::string filepath = ros::package::getPath("symbol_geometry_planner") + "/sample/HRP2JSKNTSconfig.yaml";
    std::shared_ptr<RobotConfig::RobotConfig> config = std::make_shared<RobotConfig::RobotConfig>("HRP2JSKNTS",filepath);
    BodyItemPtr robot = this->instantiate(config->get_robot());
    this->objects(std::set<BodyItemPtr>{robot});

    // load objects
    filepath = ros::package::getPath("symbol_geometry_planner") + "/sample/room73b2-73b2-ground.yaml";
    std::shared_ptr<RobotConfig::RobotConfig> groundconfig = std::make_shared<RobotConfig::RobotConfig>("",filepath);
    BodyItemPtr ground = this->instantiate(groundconfig->get_robot());
    ground->body()->rootLink()->p() = cnoid::Vector3(5.957, 0.442, 0.0);
    ground->body()->calcForwardKinematics();
    this->objects(ground);
    filepath = ros::package::getPath("symbol_geometry_planner") + "/sample/room73b2-door-right.yaml";
    std::shared_ptr<RobotConfig::RobotConfig> doorrightconfig = std::make_shared<RobotConfig::RobotConfig>("",filepath);
    BodyItemPtr doorright = this->instantiate(doorrightconfig->get_robot());
    doorright->body()->rootLink()->p() = cnoid::Vector3(0.0, -0.405, 0.0);
    doorright->body()->calcForwardKinematics();
    this->objects(doorright);
    filepath = ros::package::getPath("symbol_geometry_planner") + "/sample/room73b2-door-left.yaml";
    std::shared_ptr<RobotConfig::RobotConfig> doorleftconfig = std::make_shared<RobotConfig::RobotConfig>("",filepath);
    BodyItemPtr doorleft = this->instantiate(doorleftconfig->get_robot());
    doorleft->body()->rootLink()->p() = cnoid::Vector3(0.0, 0.405, 0.0);
    doorleft->body()->calcForwardKinematics();
    this->objects(doorleft);

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
    {
      cnoid::Position footcoords;
      footcoords.translation() = cnoid::Vector3(0.5,0,0);
      footcoords.linear() = cnoid::Matrix3(cnoid::AngleAxis(cnoid::PI,cnoid::Vector3::UnitZ()));
      Utility::fixLegToCoords(config,footcoords);
      robot->body()->calcForwardKinematics();
    }
    robot->body()->calcCenterOfMass();
    this->drawObjects();
    cnoid::msleep(500);

    std::cerr << "reset-manip-pose" << std::endl;

    // IK
    {
      // set goal
      std::vector<cnoid::Body*> variables{robot->body()};
      std::vector<std::shared_ptr<IK::IKConstraint> > tasks;
      std::vector<std::shared_ptr<IK::IKConstraint> > constraints;

      // task: lhand to target
      {
        cnoid::Position pos;
        pos.translation() = cnoid::Vector3(0.1,-0.2,0.8);
        pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(cnoid::PI,cnoid::Vector3(0,1,1).normalized()));
        tasks.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["lhand"],
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

      // constraint: door-collision
      {
        std::vector<std::shared_ptr<IK::IKConstraint> > collisionconstraints = IK::generateCollisionConstraints(config,doorrightconfig,"VClip");
        std::copy(collisionconstraints.begin(),collisionconstraints.end(),std::back_inserter(constraints));
      }
      {
        std::vector<std::shared_ptr<IK::IKConstraint> > collisionconstraints = IK::generateCollisionConstraints(config,doorleftconfig,"VClip");
        std::copy(collisionconstraints.begin(),collisionconstraints.end(),std::back_inserter(constraints));
      }

      // constraint: rleg lleg not move
      constraints.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["rleg"],
                                                                     config->get_endeffectors()["rleg"]->getlink()->T()*config->get_endeffectors()["rleg"]->getlocalpos()));
      constraints.push_back(std::make_shared<IK::PositionConstraint>(config->get_endeffectors()["lleg"],
                                                                     config->get_endeffectors()["lleg"]->getlink()->T()*config->get_endeffectors()["lleg"]->getlocalpos()));

      // constraint: support body by rleg & lleg
      {
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

      }
    }



  }
}

