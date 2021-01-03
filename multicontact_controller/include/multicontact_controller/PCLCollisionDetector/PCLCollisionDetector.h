#ifndef PCL_COLLISION_DETECTOR_H
#define PCL_COLLISION_DETECTOR_H

#include <cnoid/Body>
#include <cnoid/CollisionLinkPair>
#include <memory>
#include <pcl_ros/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/crop_box.h>

#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

namespace multicontact_controller {

  class PCLCollisionDetector {
  public:
    PCLCollisionDetector(cnoid::Body* robot);

    bool setBoxelSize(double boxelsize);

    void setObstacleModel(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model);

    bool setLinks(std::vector<cnoid::Link*>& links);

    void setAllowCollisionBoxFilters(std::map<cnoid::Link*, std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > > >& allowCollisionBoxFilters);

    // linksそれぞれについて、obstacle_modelとのcollisionを計算。
    bool solve();

    // solveした結果が入っている. collisionsのサイズは2で、0番目にはlink上の近傍点と接触方向が入る(local系), 1番目には障害物の近傍点と接触方向が入る(world系). 1番目のlinkとbodyはnullptr
    std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs() const { return collisionLinkPairs_;}

    std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    double filterDistanceGlobal() const { return filterDistanceGlobal_;}
    double& filterDistanceGlobal() { return filterDistanceGlobal_;}
    double filterDistanceLocal() const { return filterDistanceLocal_;}
    double& filterDistanceLocal() { return filterDistanceLocal_;}
  protected:
    cnoid::Body* robot_;
    std::map<const cnoid::Link*,std::shared_ptr<Vclip::Polyhedron> > collisionShapes_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model_;
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr obstacle_tree_model_;

    cnoid::LinkPtr boxelLink_;
    double boxelsize_;
    std::shared_ptr<Vclip::Polyhedron> boxelModel_;

    std::vector<std::shared_ptr<cnoid::CollisionLinkPair> > collisionLinkPairs_;
    std::vector<std::shared_ptr<cnoidbodyutils::VclipLinkPair> > vclipLinkPairs_;

    std::map<cnoid::Link*, std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > > > allowCollisionBoxFilters_;

    //params
    double filterDistanceGlobal_; //solve
    double filterDistanceLocal_; //solve

    // for visualize. choreonoidはビジュアライズ関連のメモリを開放しにくい気がする. 
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalNearObstacles_;
    cnoid::SgShapePtr boxelShape_; double boxelShapeSize_;
    cnoid::SgMaterialPtr boxelMaterial_;
  };

};

#endif
