#ifndef PCL_COLLISION_DETECTOR_ROS_ITEM_H
#define PCL_COLLISION_DETECTOR_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROS.h>

namespace cnoid {

  class PCLCollisionDetectorROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    PCLCollisionDetectorROSItem();
  };

  typedef ref_ptr<PCLCollisionDetectorROSItem> PCLCollisionDetectorROSItemPtr;
}

#endif
