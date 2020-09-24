#include "IKSampleItem.h"
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
    std::string filepath;

    filepath = ros::package::getPath("hrp2_models") + "/HRP2JSKNTS_WITH_3HAND_for_OpenHRP3/HRP2JSKNTSmain.wrl";

    BodyItemPtr robot = instantiate("HRP2JSKNTS", filepath);
    this->objects(std::set<BodyItemPtr>{robot});

    double q = 0.0;
    double dq = 0.01;

    for(double time = 0.0; time <= 6.6; time += 0.01){
      for(int j=0; j < robot->body()->numJoints(); ++j){
        robot->body()->joint(j)->q() = q;
      }
      robot->body()->calcForwardKinematics();

      this->drawObjects();

      cnoid::msleep(10);
      q += dq;
      if(fabs(q) > 0.4){
        dq = -dq;
      }

    }


    BodyItemPtr robot2 = copyObject("HRP2JSKNTS-2", robot);
    this->objects(std::set<BodyItemPtr>{robot2});

    q = 0.0;
    dq = 0.01;

    for(double time = 0.0; time <= 6.6; time += 0.01){
      for(int j=0; j < robot2->body()->numJoints(); ++j){
        robot2->body()->joint(j)->q() = q;
      }
      robot2->body()->calcForwardKinematics();

      this->drawObjects();

      cnoid::msleep(10);
      q += dq;
      if(fabs(q) > 0.4){
        dq = -dq;
      }

    }
  }
}

