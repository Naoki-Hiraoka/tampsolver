#include "IKSampleItem.h"
#include "../IK/IKsolver.h"
#include "../IK/Constraints/PositionConstraint.h"
#include "../IK/Util.h"
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
    // load robot
    std::string filepath;
    filepath = ros::package::getPath("hrp2_models") + "/HRP2JSKNTS_WITH_3HAND_for_OpenHRP3/HRP2JSKNTSmain.wrl";
    BodyItemPtr robot = instantiate("HRP2JSKNTS", filepath);
    this->objects(std::set<BodyItemPtr>{robot});

    // set EEF link()関数はjoint名を入れること
    std::vector<const cnoid::Link*> eef_links;
    std::vector<cnoid::Position> eef_localposs;
    //rleg
    {
      eef_links.push_back(robot->body()->link("RLEG_JOINT6"));
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(-0.079411,-0.01,-0.034);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(0,cnoid::Vector3(0,0,1)));
      eef_localposs.push_back(pos);
    }
    //lleg
    {
      eef_links.push_back(robot->body()->link("LLEG_JOINT6"));
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(-0.079411,0.01,-0.034);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(0,cnoid::Vector3(0,0,1)));
      eef_localposs.push_back(pos);
    }
    //rarm
    {
      eef_links.push_back(robot->body()->link("RARM_JOINT6"));
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(-0.0042,0.0392,-0.1245);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5708,cnoid::Vector3(0,1,0)));
      eef_localposs.push_back(pos);
    }
    //larm
    {
      eef_links.push_back(robot->body()->link("LARM_JOINT6"));
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(-0.0042,-0.0392,-0.1245);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(1.5708,cnoid::Vector3(0,1,0)));
      eef_localposs.push_back(pos);
    }


    // reset manip pose
    std::vector<double> reset_manip_pose{
      0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0,
        0.0, 0.0, -0.453786, 0.872665, -0.418879, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.698132,
        0.872665, -0.523599, -0.174533, -2.0944, -0.436332, -0.087266, -0.349066, 1.0472,
        0.872665, 0.523599, 0.174533, -2.0944, 0.436332, 0.087266, -0.349066, -1.0472};

    for(int j=0; j < robot->body()->numJoints(); ++j){
      robot->body()->joint(j)->q() = reset_manip_pose[j];
    }
    robot->body()->calcForwardKinematics();
    this->drawObjects();
    cnoid::msleep(500);

    std::cerr << "reset-manip-pose" << std::endl;

    // set goal
    std::vector<cnoid::BodyItemPtr> variables{robot};
    std::vector<std::shared_ptr<IK::IKConstraint> > tasks;
    std::vector<std::shared_ptr<IK::IKConstraint> > constraints;
    // task: rarm to target
    {
      cnoid::Position pos;
      pos.translation() = cnoid::Vector3(0.7,0.0,0.5);
      //pos.translation() = (eef_links[2]->T()*eef_localposs[2]).translation() + cnoid::Vector3(0.1, 0, 0);
      pos.linear() = cnoid::Matrix3(cnoid::AngleAxis(0.0,cnoid::Vector3(0,1,0)));
      //pos.linear() = (eef_links[2]->T()*eef_localposs[2]).linear();
      tasks.push_back(std::make_shared<IK::PositionConstraint>(eef_links[2],eef_localposs[2],
                                                         nullptr,pos));
    }
    // constraint: joint mim-max
    {
    std::vector<std::shared_ptr<IK::IKConstraint> > minmaxconstraints = IK::generateMinMaxConstraints(robot);
    constraints.insert(constraints.end(),minmaxconstraints.begin(),minmaxconstraints.end());
    }
    // constraint: rleg & lleg not move
    constraints.push_back(std::make_shared<IK::PositionConstraint>(eef_links[0],eef_localposs[0],
                                                         nullptr,eef_links[0]->T()*eef_localposs[0]));
    constraints.push_back(std::make_shared<IK::PositionConstraint>(eef_links[1],eef_localposs[1],
                                                         nullptr,eef_links[1]->T()*eef_localposs[1]));
    // solve ik
    IK::IKsolver solver(variables,tasks,constraints);

    solver.set_debug_level(2);
    for(size_t i=0;i<tasks.size();i++) tasks[i]->set_debug_level(2);
    for(size_t i=0;i<constraints.size();i++) constraints[i]->set_debug_level(2);

    for(size_t i=0; i<10;i++){
      solver.solve_one_loop();
      this->drawObjects();
      this->os << i << std::endl;
      cnoid::msleep(500);
    }

  }
}

