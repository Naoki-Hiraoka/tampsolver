#ifndef MULTICONTACT_FOOTCOORDS_ROS_ITEM_H
#define MULTICONTACT_FOOTCOORDS_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROS.h>

namespace cnoid {

  class MultiContactFootCoordsROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    MultiContactFootCoordsROSItem();
  };

  typedef ref_ptr<MultiContactFootCoordsROSItem> MultiContactFootCoordsROSItemPtr;
}

#endif
