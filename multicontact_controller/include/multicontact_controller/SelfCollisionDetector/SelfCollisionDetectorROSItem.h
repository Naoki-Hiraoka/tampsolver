#ifndef SELF_COLLISION_DETECTOR_ROS_ITEM_H
#define SELF_COLLISION_DETECTOR_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetectorROS.h>

namespace cnoid {

  class SelfCollisionDetectorROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    SelfCollisionDetectorROSItem();
  };

  typedef ref_ptr<SelfCollisionDetectorROSItem> SelfCollisionDetectorROSItemPtr;
}

#endif
