#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <QCoreApplication>

using namespace std::placeholders;
using namespace cnoid;

namespace cnoid {

  void ChoreonoidCppItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<ChoreonoidCppItem>("ChoreonoidCppItem");
  }

  ChoreonoidCppItem::ChoreonoidCppItem(){
    worker = std::make_shared<choreonoid_cpp::ChoreonoidCpp>();
  }

  void ChoreonoidCppItem::main()
  {
    worker->setObjectsSet([this](const std::set<cnoid::Body*>& objs){ this->objects (objs); });
    worker->setObjectsVector([this](const std::vector<cnoid::Body*>& objs){ this->objects (objs); });
    worker->setObjects([this](cnoid::Body*& obj){ this->objects (obj); });
    worker->setDrawOn(std::bind(&ChoreonoidCppItem::drawOn,this,_1,_2));
    worker->setDrawObjects(std::bind(&ChoreonoidCppItem::drawObjects,this,_1));
    worker->setFlush(std::bind(&ChoreonoidCppItem::flush,this));
    worker->hasViewer() = true;

    QStringList argv_list = QCoreApplication::arguments();
    char* argv[argv_list.size()];
    for(size_t i=0;i<argv_list.size();i++){
      argv[i] = argv_list.at(i).toUtf8().data();
    }
    worker->main(argv_list.size(),argv);
  }
}

