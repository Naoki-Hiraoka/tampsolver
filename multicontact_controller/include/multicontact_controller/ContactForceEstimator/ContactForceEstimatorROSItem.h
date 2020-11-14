#ifndef CONTACT_FORCE_ESTIMATOR_ROS_ITEM_H
#define CONTACT_FORCE_ESTIMATOR_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROS.h>

namespace cnoid {

  class ContactForceEstimatorROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ContactForceEstimatorROSItem();
  };

  typedef ref_ptr<ContactForceEstimatorROSItem> ContactForceEstimatorROSItemPtr;
}

#endif
