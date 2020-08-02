#ifndef CNOID_PDDL_SAMPLE_ITEM_H
#define CNOID_PDDL_SAMPLE_ITEM_H

#include "../PlannerBaseItem/PlannerBaseItem.h"

namespace cnoid {

  class PddlSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    PddlSampleItem();
    PddlSampleItem(const PddlSampleItem& org);
    virtual ~PddlSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<PddlSampleItem> PddlSampleItemPtr;
}

#endif
