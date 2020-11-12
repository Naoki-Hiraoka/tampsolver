#ifndef CNOID_IRTVIEWER_SAMPLE_ITEM_H
#define CNOID_IRTVIEWER_SAMPLE_ITEM_H

#include <choreonoid_cpp/PlannerBaseItem.h>

namespace cnoid {

  class IrtViewerSampleItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    IrtViewerSampleItem();
    IrtViewerSampleItem(const IrtViewerSampleItem& org);
    virtual ~IrtViewerSampleItem();

  protected:
    virtual void main() override;

  private:

  };

  typedef ref_ptr<IrtViewerSampleItem> IrtViewerSampleItemPtr;
}

#endif
