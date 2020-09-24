#ifndef CNOID_IK_SAMPLE_ITEM_H
#define CNOID_IK_SAMPLE_ITEM_H

#include "../PlannerBaseItem/PlannerBaseItem.h"

namespace cnoid {

  class IKSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    IKSampleItem();
    IKSampleItem(const IKSampleItem& org);
    virtual ~IKSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<IKSampleItem> IKSampleItemPtr;
}

#endif
