#include <cnoid/Plugin>
#include <cnoid/MenuManager>
#include <cnoid/MessageView>

#include "ViewerSampleItem.h"

using namespace cnoid;

class ViewerSamplePlugin : public Plugin
{
public:

    ViewerSamplePlugin() : Plugin("ViewerSample")
    {
      require("Body");
    }

    virtual bool initialize() override
    {
      ViewerSampleItem::initializeClass(this);
        return true;
    }

};

CNOID_IMPLEMENT_PLUGIN_ENTRY(ViewerSamplePlugin)
