#include "ViewerSampleItem.h"
#include <ros/package.h>
#include <cnoid/Sleep>
#include <QCoreApplication>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

void ViewerSampleItem::initializeClass(ExtensionManager* ext)
{
    ext->itemManager()
      .registerClass<ViewerSampleItem>("ViewerSampleItem");
      //.addCreationPanel<ViewerSampleItem>() プラグインをロードしただけで勝手にこのItemのコンストラクタが呼ばれてしまう
}


ViewerSampleItem::ViewerSampleItem()
  : os(MessageView::instance()->cout()),
    mv(MessageView::instance()),
    kill_worker(false)
{

  worker.sigTimeout().connect([&](){ main(); });
  worker.start(1000);

}


ViewerSampleItem::ViewerSampleItem(const ViewerSampleItem& org)
  : Item(org),
    os(MessageView::instance()->cout()),
    mv(MessageView::instance()),
    kill_worker(false)
{
  worker.sigTimeout().connect([&](){ main(); });
  worker.start(1000);

}



ViewerSampleItem::~ViewerSampleItem()
{
  kill_worker = true;
}


Item* ViewerSampleItem::doDuplicate() const
{
    return new ViewerSampleItem(*this);
}


SgNode* ViewerSampleItem::getScene()
{
    return 0;
}

  BodyItemPtr ViewerSampleItem::load(const char* name, const char* url)
  {
    // search for existing body items
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(this);
    for(size_t i=0; i < bodyItems.size(); ++i){
        BodyItemPtr bodyItem = bodyItems[i];
        if(bodyItem->name() == name && bodyItem->filePath() == url){
            if(!bodyItem->load(url)){
              mv->putln(std::string("OnlineViewer: Loading ") + name + " failed.");
                return nullptr;
            }
            ItemTreeView::instance()->checkItem(bodyItem, true);
            return bodyItem;
        }
    }

    // load a new body item
    BodyItemPtr bodyItem = new BodyItem();
    mv->putln(std::string("OnlineViewer: Loading ") + name + " at " + url+ ".");
    mv->flush();
    if(!bodyItem->load(url)){
      mv->putln(std::string("OnlineViewer: Loading ") + name + " failed.");
        return nullptr;
    } else {
        bodyItem->setName(name);
        this->addChildItem(bodyItem);
        ItemTreeView::instance()->checkItem(bodyItem, true);
        return bodyItem;
    }
  }

  void ViewerSampleItem::main()
  {
    worker.stop();

    if(kill_worker) return;
      std::string filepath;

      filepath = ros::package::getPath("hrp2_models") + "/HRP2JSKNTS_WITH_3HAND_for_OpenHRP3/HRP2JSKNTSmain.wrl";

      BodyItemPtr robot = load("HRP2JSKNTS", filepath.c_str());

      double q = 0.0;
      double dq = 0.01;

  for(double time = 0.0; time <= 6.6; time += 0.01){
    for(int j=0; j < robot->body()->numJoints(); ++j){
      robot->body()->joint(j)->q() = q;
    }
    robot->body()->calcForwardKinematics();

    robot->notifyKinematicStateChange();
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    //QCoreApplication::processEvents(QEventLoop::AllEvents,10);

    cnoid::msleep(10);
    q += dq;
    if(fabs(q) > 0.4){
      dq = -dq;
    }

    if(kill_worker) return;
  }


  BodyItemPtr robot2 = new BodyItem(*robot);
  robot2->setName("HRP2JSKNTS-2");
  this->addChildItem(robot2);
  ItemTreeView::instance()->checkItem(robot2, true);
  ItemList<BodyItem> bodyItems;
  bodyItems.extractChildItems(this);
  for(size_t i=0; i < bodyItems.size(); ++i){
    BodyItemPtr bodyItem = bodyItems[i];
    if(bodyItem->name() != "HRP2JSKNTS-2"){
      ItemTreeView::instance()->checkItem(bodyItem, false);
    }
  }

  q = 0.0;
  dq = 0.01;

  for(double time = 0.0; time <= 6.6; time += 0.01){
    for(int j=0; j < robot2->body()->numJoints(); ++j){
      robot2->body()->joint(j)->q() = q;
    }
    robot2->body()->calcForwardKinematics();

    robot2->notifyKinematicStateChange();
    QCoreApplication::processEvents(QEventLoop::AllEvents);
    //QCoreApplication::processEvents(QEventLoop::AllEvents,10);

    cnoid::msleep(10);
    q += dq;
    if(fabs(q) > 0.4){
      dq = -dq;
    }

    if(kill_worker) return;
  }
  }
}

