#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetector.h>

namespace multicontact_controller {

  SelfCollisionDetector::SelfCollisionDetector(cnoid::Body* robot)
    : robot_(robot)
  {
    cnoidbodyutils::convertCollisionToConvexHull(robot_);
    for(size_t i=0;i<robot_->numLinks();i++){
      cnoid::SgNode* collisionshape = robot_->link(i)->collisionShape();
      if(!collisionshape) continue;
      std::shared_ptr<Vclip::Polyhedron> vclipCollisionShape = cnoidbodyutils::convertToVClipModel(collisionshape);
      if(!vclipCollisionShape) continue;
      collisionShapes_[robot_->link(i)] = vclipCollisionShape;
    }
  }

  bool SelfCollisionDetector::solve(){
    bool collisionLinkPairsChanged = false;
    if(vclipLinkPairs_.size() != collisionLinkPairs_.size()){
      vclipLinkPairs_.resize(collisionLinkPairs_.size());//for visualization
      collisionLinkPairsChanged = true;
    }

    for(size_t i=0;i<collisionLinkPairs_.size();i++){
      std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = collisionLinkPairs_[i];
      if(!collisionLinkPair || !collisionLinkPair->link[0] || !collisionLinkPair->link[1]){
        std::cerr << "CollisionLinkPair is not valid" << std::endl;
        return false;
      }

      cnoid::Vector3 p0, p1;
      if(collisionLinkPairsChanged ||
         !vclipLinkPairs_[i] ||
         collisionLinkPair->link[0] != vclipLinkPairs_[i]->link0() ||
         collisionLinkPair->link[1] != vclipLinkPairs_[i]->link1()){
        cnoid::Link* link0 = collisionLinkPair->link[0];
        cnoid::Link* link1 = collisionLinkPair->link[1];

        if(collisionShapes_.find(link0) == collisionShapes_.end()){
          std::cerr << link0->name() << "not found" << std::endl;
          return false;
        }
        if(collisionShapes_.find(link1) == collisionShapes_.end()){
          std::cerr << link1 << "not found" << std::endl;
          return false;
        }
        std::shared_ptr<Vclip::Polyhedron> model0 = collisionShapes_[link0];
        std::shared_ptr<Vclip::Polyhedron> model1 = collisionShapes_[link1];

        vclipLinkPairs_[i] = std::make_shared<cnoidbodyutils::VclipLinkPair>(link0, model0, link1, model1);
      }
      std::shared_ptr<cnoidbodyutils::VclipLinkPair> vclipLinkPair = vclipLinkPairs_[i];
      double distance = vclipLinkPair->computeDistance(p0,p1);
      double sign = (distance >= 0)? 1.0 : -1.0;

      collisionLinkPair->collisions.resize(2);
      collisionLinkPair->collisions[0].point = collisionLinkPair->link[0]->T().inverse() * p0;
      collisionLinkPair->collisions[0].normal = collisionLinkPair->link[0]->R().inverse() * sign * (p1 - p0).normalized();
      collisionLinkPair->collisions[0].depth = - distance;
      collisionLinkPair->collisions[1].point = collisionLinkPair->link[1]->T().inverse() * p1;
      collisionLinkPair->collisions[1].normal = collisionLinkPair->link[1]->R().inverse() * sign * (p0 - p1).normalized();
      collisionLinkPair->collisions[1].depth = - distance;

    }

    return true;
  }

  std::vector<cnoid::SgNodePtr> SelfCollisionDetector::getDrawOnObjects(){
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    for(size_t i=0;i<vclipLinkPairs_.size();i++){
      std::vector<cnoid::SgNodePtr> objects = vclipLinkPairs_[i]->getDrawOnObjects();
      std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
    }
    return drawOnObjects;
  }
};
