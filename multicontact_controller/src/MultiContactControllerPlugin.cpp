#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROSItem.h>
#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROSItem.h>
#include <multicontact_controller/PWTController/PWTControllerROSItem.h>
#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetectorROSItem.h>
#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROSItem.h>
#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROSItem.h>

using namespace cnoid;

class MultiContactControllerPlugin : public Plugin
{
public:

    MultiContactControllerPlugin() : Plugin("MultiContactController")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ContactForceEstimatorROSItem::initializeClass(this);
      MultiContactFootCoordsROSItem::initializeClass(this);
      PWTControllerROSItem::initializeClass(this);
      SelfCollisionDetectorROSItem::initializeClass(this);
      PCLCollisionDetectorROSItem::initializeClass(this);
      ContactBreakAbilityCheckerROSItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactControllerPlugin)
