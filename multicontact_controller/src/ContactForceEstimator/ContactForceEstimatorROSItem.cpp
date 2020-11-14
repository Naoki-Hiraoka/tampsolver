#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROSItem.h>

namespace cnoid {

  void ContactForceEstimatorROSItem::initializeClass(ExtensionManager* ext)
  {
    ext->itemManager()
      .registerClass<ContactForceEstimatorROSItem>("ContactForceEstimatorROSItem");
  }

  ContactForceEstimatorROSItem::ContactForceEstimatorROSItem(){
    worker = std::make_shared<multicontact_controller::ContactForceEstimatorROS>();
  }

}

