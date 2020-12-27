#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROSItem.h>

namespace cnoid {

  void PCLCollisionDetectorROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<PCLCollisionDetectorROSItem>("PCLCollisionDetectorROSItem");
  }

  PCLCollisionDetectorROSItem::PCLCollisionDetectorROSItem(){
    worker = std::make_shared<multicontact_controller::PCLCollisionDetectorROS>();
  }

}

