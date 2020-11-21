#include <multicontact_controller/WalkController/WalkControllerROSItem.h>

namespace cnoid {

  void WalkControllerROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<WalkControllerROSItem>("WalkControllerROSItem");
  }

  WalkControllerROSItem::WalkControllerROSItem(){
    worker = std::make_shared<multicontact_controller::WalkControllerROS>();
  }

}

