#ifndef CHOREONOID_CPP_H
#define CHOREONOID_CPP_H

#include <cnoid/SceneProvider>
//#include <cnoid/SceneView>
#include <cnoid/BodyItem>
#include <cnoid/SceneCollision>
#include <cnoid/CollisionSeq>
#include <cnoid/Sleep>
#include <cnoid/Timer>
#include <functional>

namespace choreonoid_cpp {

  class ChoreonoidCpp
  {
  public:
    void setObjectsSet(std::function<void(const std::set<cnoid::Body*>&)> f) { objects_set_ = f;}
    void setObjectsVector(std::function<void(const std::vector<cnoid::Body*>&)> f) {objects_vector_ = f; }
    void setObjects(std::function<void(cnoid::Body*&)> f) {objects_ = f;}
    void setDrawOn(std::function<void(cnoid::SgNodePtr, bool)> f) {drawon_ = f;}
    void setDrawObjects(std::function<void(bool)> f) {drawobjects_ = f; }
    void setFlush(std::function<void()> f) {flush_ = f; }

    bool viewer() const {return viewer_; }
    bool& viewer() {return viewer_; }

    virtual void main() {};

  protected:
    void objects(const std::set<cnoid::Body*>& objs);
    void objects(const std::vector<cnoid::Body*>& objs);
    void objects(cnoid::Body*& obj);
    void drawOn(cnoid::SgNodePtr obj, bool flush=false);
    void drawObjects(bool flush=true);
    void flush();

    bool viewer_=false;

  private:
    std::function<void(const std::set<cnoid::Body*>&)> objects_set_;
    std::function<void(const std::vector<cnoid::Body*>&)> objects_vector_;
    std::function<void(cnoid::Body*&)> objects_;
    std::function<void(cnoid::SgNodePtr, bool)> drawon_;
    std::function<void(bool)> drawobjects_;
    std::function<void()> flush_;

  };

}

#endif
