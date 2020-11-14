#ifndef CHOREONOID_CPP_ITEM_H
#define CHOREONOID_CPP_ITEM_H

#include <choreonoid_cpp/PlannerBaseItem.h>
#include <choreonoid_cpp/ChoreonoidCpp.h>

namespace cnoid {

  class ChoreonoidCppItem : public PlannerBaseItem
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    ChoreonoidCppItem();

  protected:
    virtual void main() override;

    std::shared_ptr<choreonoid_cpp::ChoreonoidCpp> worker;
  };

  typedef ref_ptr<ChoreonoidCppItem> ChoreonoidCppItemPtr;
}

#endif
