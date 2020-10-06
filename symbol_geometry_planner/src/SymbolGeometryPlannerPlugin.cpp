#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "PlannerBaseItem/PlannerBaseItem.h"

#include "IrtViewerSampleItem/IrtViewerSampleItem.h"
#include "PddlSampleItem/PddlSampleItem.h"
#include "RoboptimSampleItem/RoboptimSampleItem.h"
#include "RoboptimSparseSampleItem/RoboptimSparseSampleItem.h"
#include "IKSampleItem/IKSampleItem.h"
#include "MultiContactIKSampleItem/MultiContactIKSampleItem.h"

using namespace cnoid;

class SymbolGeometryPlannerPlugin : public Plugin
{
public:

    SymbolGeometryPlannerPlugin() : Plugin("SymbolGeometryPlanner")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      PlannerBaseItem::initializeClass(this);

      IrtViewerSampleItem::initializeClass(this);
      PddlSampleItem::initializeClass(this);
      RoboptimSampleItem::initializeClass(this);
      RoboptimSparseSampleItem::initializeClass(this);
      IKSampleItem::initializeClass(this);
      MultiContactIKSampleItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SymbolGeometryPlannerPlugin)
