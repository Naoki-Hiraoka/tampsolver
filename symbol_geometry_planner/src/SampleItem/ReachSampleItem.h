#ifndef CNOID_REACH_SAMPLE_ITEM_H
#define CNOID_REACH_SAMPLE_ITEM_H

#include <choreonoid_cpp/PlannerBaseItem.h>

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
