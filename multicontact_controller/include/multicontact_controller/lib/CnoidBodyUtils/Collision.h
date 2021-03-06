#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_COLLISION_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_COLLISION_H

#include <cnoid/SceneDrawables>
#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <cnoid/JointPath>
#include <Eigen/Sparse>
#include <multicontact_controller_msgs/CollisionArray.h>

namespace Vclip{
  class Polyhedron;
  class FeaturePair;
}

namespace multicontact_controller{
  namespace cnoidbodyutils{
    cnoid::SgMesh* convertToSgMesh(cnoid::SgNode* collisionshape);

    cnoid::SgNode* convertToConvexHull(cnoid::SgNode* collisionshape);

    void convertCollisionToConvexHull(cnoid::Body* robot);

    std::shared_ptr<Vclip::Polyhedron> convertToVClipModel(cnoid::SgNode* collisionshape);

    class VclipLinkPair {
    public:
      VclipLinkPair(const cnoid::Link* link0, std::shared_ptr<Vclip::Polyhedron> pqp_model0, const cnoid::Link* link1, std::shared_ptr<Vclip::Polyhedron> pqp_model1, double tolerance=0);
      bool checkCollision();
      double computeDistance(cnoid::Vector3& q1, cnoid::Vector3& q2);//q1,q2はworld系
      double computeDistanceLocal(cnoid::Vector3& q1, cnoid::Vector3& q2);//q1,q2はlocal系
      const cnoid::Link* link0() { return link1_; }
      const cnoid::Link* link1() { return link2_; }
      double getTolerance() { return tolerance_; }
      void setTolerance(double t) { tolerance_ = t; }

      std::vector<cnoid::SgNodePtr> getDrawOnObjects();
    private:
      const cnoid::Link* link1_;
      const cnoid::Link* link2_;
      std::shared_ptr<Vclip::Polyhedron> Vclip_Model1, Vclip_Model2;
      std::shared_ptr<Vclip::FeaturePair> Feature_Pair;
      double tolerance_;

      // for visualize
      cnoid::Vector3 q1_, q2_;
      double distance_;
      cnoid::SgLineSetPtr lines;
    };

    class Collision {
    public:
      Collision(cnoid::Body* robot);

      // world座標系 Ax = b, dl <= Cx <= du. xの次元数は[rootlink + numJoints]
      void getCollisionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc);

      std::vector<cnoid::SgNodePtr> getDrawOnObjects();

      cnoid::CollisionLinkPair& collisionLinkPair() { return collisionLinkPair_;}
      cnoid::CollisionLinkPair collisionLinkPair() const { return collisionLinkPair_;}
      double& tolerance() { return tolerance_;}//これ以下に接近することを許さない
      double tolerance() const { return tolerance_;}
      double& maxDistance() { return maxDistance_;}//これ以上離れている場合考慮しない
      double maxDistance() const { return maxDistance_;}
    protected:
      cnoid::Body* robot_;

      cnoid::CollisionLinkPair collisionLinkPair_;
      double tolerance_;
      double maxDistance_;

      // for visualization
      cnoid::SgLineSetPtr lines_;

      // cache
      cnoid::JointPath path_;
      cnoid::JointPath path0_;
      cnoid::JointPath path1_;
    };

    //collisionLinkPair.collisionのサイズは2.
    void collisionLinkPairToMsg(const std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair, multicontact_controller_msgs::Collision& msg, const ros::Time& stamp=ros::Time(0), size_t seq=0);
    void collisionMsgToLinkPair(cnoid::Body* robot, const multicontact_controller_msgs::Collision& msg, cnoid::CollisionLinkPair& collisionLinkPair);

    void collisionArrayMsgToCnoid(cnoid::Body* robot, const multicontact_controller_msgs::CollisionArray& msg, std::vector<std::shared_ptr<Collision> >& collisions);

    std::vector<cnoid::SgNodePtr> collisionLinkPairToDrawOnObjects(const cnoid::CollisionLinkPair& collisionLinkPair, double maxDistance, double tolerance, cnoid::SgLineSetPtr& lines);
    std::vector<cnoid::SgNodePtr> collisionLinkPairToDrawOnObjects(const cnoid::CollisionLinkPair& collisionLinkPair, double maxDistance=10.0, double tolerance=0.0);

    void calcCurrentDistance(const cnoid::CollisionLinkPair& collisionLinkPair, cnoid::Vector3& p0, cnoid::Vector3& p1, double& distance, cnoid::Vector3& normal01);
  };
};

#endif
