#ifndef CNOID_ROBOPTIM_SPARSE_SAMPLE_ITEM_H
#define CNOID_ROBOPTIM_SPARSE_SAMPLE_ITEM_H

#include "../PlannerBaseItem/PlannerBaseItem.h"

namespace cnoid {

  class RoboptimSparseSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    RoboptimSparseSampleItem();
    RoboptimSparseSampleItem(const RoboptimSparseSampleItem& org);
    virtual ~RoboptimSparseSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<RoboptimSparseSampleItem> RoboptimSparseSampleItemPtr;
}

#endif
