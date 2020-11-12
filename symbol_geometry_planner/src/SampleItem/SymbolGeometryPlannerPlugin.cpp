#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "IrtViewerSampleItem.h"
#include "PddlSampleItem.h"
#include "RoboptimSampleItem.h"
#include "RoboptimSparseSampleItem.h"
#include "IKSampleItem.h"
#include "MultiContactIKSampleItem.h"
#include "ReachSampleItem.h"

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
      IrtViewerSampleItem::initializeClass(this);
      PddlSampleItem::initializeClass(this);
      RoboptimSampleItem::initializeClass(this);
      RoboptimSparseSampleItem::initializeClass(this);
      IKSampleItem::initializeClass(this);
      MultiContactIKSampleItem::initializeClass(this);
      ReachSampleItem::initializeClass(this);
      return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(SymbolGeometryPlannerPlugin)
