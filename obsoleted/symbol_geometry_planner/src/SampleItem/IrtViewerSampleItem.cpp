#include "IrtViewerSampleItem.h"
#include <ros/package.h>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

  void IrtViewerSampleItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<IrtViewerSampleItem>("IrtViewerSampleItem");
  }


  IrtViewerSampleItem::IrtViewerSampleItem()
    : PlannerBaseItem()
  {
  }


  IrtViewerSampleItem::IrtViewerSampleItem(const IrtViewerSampleItem& org)
    : PlannerBaseItem(org)
  {
  }



  IrtViewerSampleItem::~IrtViewerSampleItem()
  {
  }


  void IrtViewerSampleItem::main()
  {
    std::string filepath;

    filepath = ros::package::getPath("hrp2_models") + "/HRP2JSKNTS_WITH_3HAND_for_OpenHRP3/HRP2JSKNTSmain.wrl";

    BodyItemPtr robot = instantiate("HRP2JSKNTS", filepath);
    this->objects(std::set<BodyItemPtr>{robot});

    SgLineSetPtr lines = new SgLineSet;
    lines->setLineWidth(1.0);
    lines->getOrCreateVertices()->resize(2);
    lines->getOrCreateVertices()->at(0) = cnoid::Vector3f(0.0,0.0,0.0);
    lines->getOrCreateVertices()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
    lines->addLine(0,1);
    lines->getOrCreateColors()->resize(1);
    lines->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,0.0,0.0);
    lines->colorIndices().resize(2);
    lines->colorIndices().at(0) = 0;
    lines->colorIndices().at(1) = 0;

    double q = 0.0;
    double dq = 0.01;

    for(double time = 0.0; time <= 6.6; time += 0.01){
      for(int j=0; j < robot->body()->numJoints(); ++j){
        robot->body()->joint(j)->q() = q;
      }
      robot->body()->calcForwardKinematics();

      lines->getOrCreateVertices()->at(1) = cnoid::Vector3f(cos(q),sin(q),0.0);

      this->drawObjects(false);
      this->drawOn(lines);
      this->flush();

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

