#ifndef CNOID_PLANNER_BASE_ITEM_H
#define CNOID_PLANNER_BASE_ITEM_H

#include <cnoid/Item>
#include <cnoid/SceneProvider>
#include <cnoid/ItemManager>
#include <cnoid/MessageView>
#include <cnoid/BodyItem>
#include <cnoid/ItemTreeView>
#include <cnoid/RootItem>
#include <cnoid/WorldItem>
#include <cnoid/SceneCollision>
#include <cnoid/CollisionSeq>
#include <cnoid/Sleep>
#include <cnoid/Timer>
#include <sstream>

namespace cnoid {

  class PlannerBaseItem : public Item, public SceneProvider
  {
  public:
    static void initializeClass(ExtensionManager* ext);

    PlannerBaseItem();
    PlannerBaseItem(const PlannerBaseItem& org);
    virtual ~PlannerBaseItem();

  protected:
    virtual Item* doDuplicate() const override;
    virtual SgNode* getScene() override;

    BodyItemPtr instantiate(const std::string& name, const std::string& url);
    BodyItemPtr copyObject(const std::string& name, const BodyItemPtr obj);
    void objects(const std::set<BodyItemPtr>& objs);
    void objects(const std::vector<BodyItemPtr>& objs);
    void objects(const BodyItemPtr& obj);
    void drawObjects();

    std::ostream& os;
    MessageView* mv;

    virtual void main();

  private:

    // 長時間かかる処理で，内部でsignalを出す場合
    // std::threadやQThreadで別スレッドで処理するとエラーになる(QBasicTimer::start: Timers cannot be started from another thread)
    // eventとして同一スレッドで処理する場合，choreonoid全体の起動が終わっていない段階で処理が始まってしまうと以後の起動処理が止まってしまうため，timerを用いてevent開始のタイミングを遅延させる．eventが始まったらtimerを止めること
    // 通常は一つのsignaleventの処理が完了してから次のeventの処理が始まるが，event中にQCoreApplication::processEvents()を呼ぶとその瞬間にevent処理が割り込まれる．
    Timer _worker;
    std::set<BodyItemPtr> _objects;
  };

  typedef ref_ptr<PlannerBaseItem> PlannerBaseItemPtr;
}

#endif
