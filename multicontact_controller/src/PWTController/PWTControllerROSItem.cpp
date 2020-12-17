#include <multicontact_controller/PWTController/PWTControllerROSItem.h>

namespace cnoid {

  void PWTControllerROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<PWTControllerROSItem>("PWTControllerROSItem");
  }

  PWTControllerROSItem::PWTControllerROSItem(){
    worker = std::make_shared<multicontact_controller::PWTControllerROS>();
  }

}

