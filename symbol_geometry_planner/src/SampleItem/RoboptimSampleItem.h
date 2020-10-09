#ifndef CNOID_ROBOPTIM_SAMPLE_ITEM_H
#define CNOID_ROBOPTIM_SAMPLE_ITEM_H

#include "PlannerBaseItem.h"

namespace cnoid {

  class RoboptimSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    RoboptimSampleItem();
    RoboptimSampleItem(const RoboptimSampleItem& org);
    virtual ~RoboptimSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<RoboptimSampleItem> RoboptimSampleItemPtr;
}

#endif
