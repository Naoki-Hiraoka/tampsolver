#ifndef WALK_CONTROLLER_ROS_ITEM_H
#define WALK_CONTROLLER_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/WalkController/WalkControllerROS.h>

namespace cnoid {

  class WalkControllerROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    WalkControllerROSItem();
  };

  typedef ref_ptr<WalkControllerROSItem> WalkControllerROSItemPtr;
}

#endif
