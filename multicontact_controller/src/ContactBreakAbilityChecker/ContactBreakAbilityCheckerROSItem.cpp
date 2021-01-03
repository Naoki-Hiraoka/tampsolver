#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROSItem.h>

namespace cnoid {

  void ContactBreakAbilityCheckerROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<ContactBreakAbilityCheckerROSItem>("ContactBreakAbilityCheckerROSItem");
  }

  ContactBreakAbilityCheckerROSItem::ContactBreakAbilityCheckerROSItem(){
    worker = std::make_shared<multicontact_controller::ContactBreakAbilityCheckerROS>();
  }

}

