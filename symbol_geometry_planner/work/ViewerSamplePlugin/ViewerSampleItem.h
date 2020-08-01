#ifndef CNOID_VIEWER_SAMPLE_ITEM_H
#define CNOID_VIEWER_SAMPLE_ITEM_H

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

#include <cnoid/Timer>

#include <sstream>

namespace cnoid {

class ViewerSampleItem : public Item, public SceneProvider
{
public:
    static void initializeClass(ExtensionManager* ext);

    ViewerSampleItem();
    ViewerSampleItem(const ViewerSampleItem& org);
    virtual ~ViewerSampleItem();

  void main();

  BodyItemPtr load(const char* name, const char* url);

protected:
    virtual Item* doDuplicate() const override;
    virtual SgNode* getScene() override;

private:

  std::ostream& os;
  MessageView* mv;
  bool kill_worker;

  // 長時間かかる処理で，内部でsignalを出す場合
  // std::threadやQThreadで別スレッドで処理するとエラーになる(QBasicTimer::start: Timers cannot be started from another thread)
  // eventとして同一スレッドで処理する場合，choreonoid全体の起動が終わっていない段階で処理が始まってしまうと以後の起動処理が止まってしまうため，timerを用いてevent開始のタイミングを遅延させる．eventが始まったらtimerを止めること
  // 通常は一つのsignaleventの処理が完了してから次のeventの処理が始まるが，event中にQCoreApplication::processEvents()を呼ぶとその瞬間にevent処理が割り込まれる．
  Timer worker;

};

typedef ref_ptr<ViewerSampleItem> ViewerSampleItemPtr;
}

#endif
