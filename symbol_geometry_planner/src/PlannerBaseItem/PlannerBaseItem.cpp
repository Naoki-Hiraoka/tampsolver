#include "PlannerBaseItem.h"

#include <QCoreApplication>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

  void PlannerBaseItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<PlannerBaseItem>("PlannerBaseItem");
    //.addCreationPanel<ViewerSampleItem>() プラグインをロードしただけで勝手にこのItemのコンストラクタが呼ばれてしまう
  }


  PlannerBaseItem::PlannerBaseItem()
    : os(MessageView::instance()->cout()),
      mv(MessageView::instance())
  {
    this->_markerGroup = new SgGroup;
    this->_markerGroup->setName("Marker");
    SceneView::instance()->sceneWidget()->sceneRoot()->addChild(this->_markerGroup);

    this->_worker.sigTimeout().connect([&](){ this->main_common(); });
    this->_worker.start(100);

  }


  PlannerBaseItem::PlannerBaseItem(const PlannerBaseItem& org)
    : Item(org),
      os(MessageView::instance()->cout()),
      mv(MessageView::instance())
  {
    this->_worker.sigTimeout().connect([&](){ this->main_common(); });
    this->_worker.start(100);

  }



  PlannerBaseItem::~PlannerBaseItem()
  {
  }


  Item* PlannerBaseItem::doDuplicate() const
  {
    return new PlannerBaseItem(*this);
  }


  SgNode* PlannerBaseItem::getScene()
  {
    return 0;
  }


  BodyItemPtr PlannerBaseItem::instantiate(const std::string& name, const std::string& url)
  {
    // search for existing body items
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(this);
    for(size_t i=0; i < bodyItems.size(); ++i){
      BodyItemPtr bodyItem = bodyItems[i];
      if(bodyItem->name() == name && bodyItem->filePath() == url){
        // if(!bodyItem->load(url)){
        //   mv->putln(std::string("OnlineViewer: Loading ") + name + " failed.");
        //   return nullptr;
        // }
        return bodyItem;
      }
    }

    // load a new body item
    BodyItemPtr bodyItem = new BodyItem();
    mv->putln(std::string("OnlineViewer: Loading ") + name + " at " + url+ ".");
    mv->flush();
    if(!bodyItem->load(url)){
      mv->putln(MessageView::WARNING, std::string("OnlineViewer: Loading ") + name + " failed.");
      return nullptr;
    } else {
      bodyItem->setName(name);
      this->addChildItem(bodyItem);
      return bodyItem;
    }
  }


  BodyItemPtr PlannerBaseItem::copyObject(const std::string& name, const BodyItemPtr obj)
  {
    // search for existing body items
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(this);
    for(size_t i=0; i < bodyItems.size(); ++i){
      BodyItemPtr bodyItem = bodyItems[i];
      if(bodyItem->name() == name && bodyItem->filePath() == obj->filePath()){
        // if(!bodyItem->load(url)){
        //   mv->putln(std::string("OnlineViewer: Loading ") + name + " failed.");
        //   return nullptr;
        // }
        return bodyItem;
      }
    }

    // copy body item
    BodyItemPtr bodyItem = new BodyItem(*obj);
    bodyItem->setName(name);
    this->addChildItem(bodyItem);
    return bodyItem;
  }


  void PlannerBaseItem::objects(const std::set<BodyItemPtr>& objs)
  {
    this->_objects.clear();

    // search for existing body items
    ItemList<BodyItem> bodyItems;
    bodyItems.extractChildItems(this);
    for(size_t i=0; i < bodyItems.size(); ++i){
      BodyItemPtr bodyItem = bodyItems[i];
      if(objs.find(bodyItem) != objs.end()){
        ItemTreeView::instance()->checkItem(bodyItem, true);
        this->_objects.insert(bodyItem);
      } else {
        ItemTreeView::instance()->checkItem(bodyItem, false);
      }
    }
  }


  void PlannerBaseItem::objects(const std::vector<BodyItemPtr>& objs)
  {
    objects(std::set<BodyItemPtr>(objs.begin(), objs.end()));
  }


  void PlannerBaseItem::objects(const BodyItemPtr& obj)
  {
    ItemTreeView::instance()->checkItem(obj, true);
    this->_objects.insert(obj);
  }

  void PlannerBaseItem::drawOn(cnoid::SgNodePtr obj, bool flush)
  {
    if(this->_markerGroup->addChildOnce(obj)){
      obj->notifyUpdate();
    }
    if(flush){
      this->flush();
    }
  }

  void PlannerBaseItem::drawObjects(bool flush)
  {
    this->_markerGroup->clearChildren();

    for(std::set<BodyItemPtr>::iterator iter = this->_objects.begin(); iter != this->_objects.end(); iter++){
      (*iter)->notifyKinematicStateChange();
    }
    if(flush){
      this->flush();
    }
  }

  void PlannerBaseItem::flush()
  {
    QCoreApplication::processEvents(QEventLoop::AllEvents);
  }

  void PlannerBaseItem::main_common()
  {
    this->_worker.stop();
    this->main();
  }

  void PlannerBaseItem::main()
  {
  }

}

