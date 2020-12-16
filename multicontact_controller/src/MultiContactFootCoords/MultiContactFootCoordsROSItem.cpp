#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROSItem.h>

namespace cnoid {

  void MultiContactFootCoordsROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<MultiContactFootCoordsROSItem>("MultiContactFootCoordsROSItem");
  }

  MultiContactFootCoordsROSItem::MultiContactFootCoordsROSItem(){
    worker = std::make_shared<multicontact_controller::MultiContactFootCoordsROS>();
  }

}

