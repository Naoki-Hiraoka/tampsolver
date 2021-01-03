#ifndef CONTACT_BREAKABILITY_CHECKER_ROS_ITEM_H
#define CONTACT_BREAKABILITY_CHECKER_ROS_ITEM_H

#include <choreonoid_cpp/ChoreonoidCppItem.h>
#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROS.h>

namespace cnoid {

  class ContactBreakAbilityCheckerROSItem : public ChoreonoidCppItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ContactBreakAbilityCheckerROSItem();
  };

  typedef ref_ptr<ContactBreakAbilityCheckerROSItem> ContactBreakAbilityCheckerROSItemPtr;
}

#endif
