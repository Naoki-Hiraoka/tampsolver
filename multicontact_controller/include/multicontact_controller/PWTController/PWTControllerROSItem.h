#ifndef PWT_CONTROLLER_ROS_ITEM_H
#define PWT_CONTROLLER_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/PWTController/PWTControllerROS.h>

namespace cnoid {

  class PWTControllerROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    PWTControllerROSItem();
  };

  typedef ref_ptr<PWTControllerROSItem> PWTControllerROSItemPtr;
}

#endif
