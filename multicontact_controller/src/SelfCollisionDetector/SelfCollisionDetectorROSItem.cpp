#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetectorROSItem.h>

namespace cnoid {

  void SelfCollisionDetectorROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<SelfCollisionDetectorROSItem>("SelfCollisionDetectorROSItem");
  }

  SelfCollisionDetectorROSItem::SelfCollisionDetectorROSItem(){
    worker = std::make_shared<multicontact_controller::SelfCollisionDetectorROS>();
  }

}

