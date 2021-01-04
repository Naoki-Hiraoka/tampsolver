#ifndef SELF_COLLISION_DETECTOR_H
#define SELF_COLLISION_DETECTOR_H

#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <memory>

#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {
  void setupCollisionPairFromParam(cnoid::Body* robot, std::vector<std::shared_ptr<cnoid::CollisionLinkPair> >& collisionLinkPairs);

  class SelfCollisionDetector {
  public:
    SelfCollisionDetector(cnoid::Body* robot);

    // collisionLinkPairそれぞれについて、collisionを計算。collisionsのサイズは2で、各link上の近傍点と法線が入る(world系)
    bool solve();

    std::vector<std::shared_ptr<cnoid::CollisionLinkPair> >& collisionLinkPairs() { return collisionLinkPairs_;}
    std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs() const { return collisionLinkPairs_;}

    std::vector<cnoid::SgNodePtr> getDrawOnObjects();
  protected:
    cnoid::Body* robot_;
    std::map<cnoid::Link*,std::shared_ptr<Vclip::Polyhedron> > collisionShapes_;

    std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs_;
    std::vector<std::shared_ptr<cnoidbodyutils::VclipLinkPair> > vclipLinkPairs_;
  };

};

#endif
