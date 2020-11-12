#ifndef CNOID_MULTICONTACTIK_SAMPLE_ITEM_H
#define CNOID_MULTICONTACTIK_SAMPLE_ITEM_H

#include <choreonoid_cpp/PlannerBaseItem.h>

namespace cnoid {

  class MultiContactIKSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    MultiContactIKSampleItem();
    MultiContactIKSampleItem(const MultiContactIKSampleItem& org);
    virtual ~MultiContactIKSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<MultiContactIKSampleItem> MultiContactIKSampleItemPtr;
}

#endif
