#ifndef CNOID_REACH_SAMPLE_ITEM_H
#define CNOID_REACH_SAMPLE_ITEM_H

#include "PlannerBaseItem.h"

namespace cnoid {

  class ReachSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<ReachSampleItem> ReachSampleItemPtr;
}

#endif
