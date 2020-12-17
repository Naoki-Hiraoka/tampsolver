#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROSItem.h>
#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROSItem.h>
#include <multicontact_controller/PWTController/PWTControllerROSItem.h>

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
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(MultiContactControllerPlugin)
