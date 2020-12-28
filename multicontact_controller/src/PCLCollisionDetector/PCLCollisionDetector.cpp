#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetector.h>

#include <cnoid/MeshGenerator>
#include <cnoid/SceneMarkers>
#include <pcl/filters/extract_indices.h>

namespace multicontact_controller {

  PCLCollisionDetector::PCLCollisionDetector(cnoid::Body* robot)
    : robot_(robot),
      obstacle_model_(new pcl::PointCloud<pcl::PointXYZ>()),
      obstacle_tree_model_(new pcl::KdTreeFLANN<pcl::PointXYZ>),
      boxelLink_(new cnoid::Link()),
      filterDistanceGlobal_(1.5),
      filterDistanceLocal_(0.5),
      globalNearObstacles_( new pcl::PointCloud<pcl::PointXYZ>() ),
      boxelsize_(0.02)
  {
    cnoidbodyutils::convertCollisionToConvexHull(robot_);
    for(size_t i=0;i<robot_->numLinks();i++){
      cnoid::SgNode* collisionshape = robot_->link(i)->collisionShape();
      if(!collisionshape) continue;
      std::shared_ptr<Vclip::Polyhedron> vclipCollisionShape = cnoidbodyutils::convertToVClipModel(collisionshape);
      if(!vclipCollisionShape) continue;
      collisionShapes_[robot_->link(i)] = vclipCollisionShape;
    }

    this->setBoxelSize(boxelsize_);
    this->setObstacleModel(obstacle_model_);
  }

  bool PCLCollisionDetector::setBoxelSize(double boxelsize){
    if(boxelsize <= 0){
      std::cerr << "boxelsize <= 0" << std::endl;
      return false;
    }
    boxelsize_ = boxelsize;

    cnoid::MeshGenerator meshGenerator;
    cnoid::SgShape* newShape(new cnoid::SgShape);
    newShape->setMesh(meshGenerator.generateBox(cnoid::Vector3(boxelsize_,boxelsize_,boxelsize_)));
    boxelLink_->setCollisionShape(newShape);
    boxelModel_ = cnoidbodyutils::convertToVClipModel(boxelLink_->collisionShape());
    for(size_t i=0;i<vclipLinkPairs_.size();i++){
      vclipLinkPairs_[i] = std::make_shared<cnoidbodyutils::VclipLinkPair>(vclipLinkPairs_[i]->link0(),collisionShapes_[vclipLinkPairs_[i]->link0()],boxelLink_,boxelModel_);
    }

    return true;
  }

  void PCLCollisionDetector::setObstacleModel(pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle_model){
    obstacle_model_ = obstacle_model;
    if(obstacle_model_->points.size() > 0){
      // Cannot create a KDTree with an empty input cloud
      obstacle_tree_model_->setInputCloud(obstacle_model_);
    }
  }

  bool PCLCollisionDetector::setLinks(std::vector<cnoid::Link*>& links){
    collisionLinkPairs_.resize(links.size());
    vclipLinkPairs_.resize(links.size());

    for(size_t i=0;i<links.size();i++){
      std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = collisionLinkPairs_[i];
      std::shared_ptr<cnoidbodyutils::VclipLinkPair>& vclipLinkPair = vclipLinkPairs_[i];

      if(!collisionLinkPair || collisionLinkPair->link[0] != links[i]){
        collisionLinkPair = std::make_shared<cnoid::CollisionLinkPair>();
        collisionLinkPair->body[0] = robot_;
        collisionLinkPair->body[1] = nullptr;
        collisionLinkPair->link[0] = links[i];
        collisionLinkPair->link[1] = nullptr;
        collisionLinkPair->collisions.resize(2);
      }
      if(!vclipLinkPair || vclipLinkPair->link0() != links[i]){
        if(collisionShapes_.find(links[i]) == collisionShapes_.end()){
          std::cerr << links[i]->name() << " not found" << std::endl;
          return false;
        }
        vclipLinkPair = std::make_shared<cnoidbodyutils::VclipLinkPair>(links[i],collisionShapes_[links[i]],boxelLink_,boxelModel_);
      }
    }
  }

  void PCLCollisionDetector::setAllowCollisionBoxFilters(std::map<cnoid::Link*, std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > > >& allowCollisionBoxFilters){
    allowCollisionBoxFilters_ = allowCollisionBoxFilters;
  }

  bool PCLCollisionDetector::solve(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr globalNearObstacles( new pcl::PointCloud<pcl::PointXYZ>() );
    pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr globalNearObstaclesTreeModel(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    if(obstacle_model_->points.size() > 0){      // Cannot create a KDTree with an empty input cloud
      pcl::PointXYZ center;
      center.getVector3fMap() = robot_->rootLink()->p().cast<float>();
      pcl::PointIndices::Ptr globalNearIndices(new pcl::PointIndices);
      std::vector<float> distances;
      obstacle_tree_model_->radiusSearch(center, filterDistanceGlobal_, globalNearIndices->indices, distances);
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(obstacle_model_);
      extract.setIndices(globalNearIndices);
      extract.setNegative(false);
      extract.filter(*globalNearObstacles);
      if(globalNearObstacles->points.size()>0){      // Cannot create a KDTree with an empty input cloud
        globalNearObstaclesTreeModel->setInputCloud(globalNearObstacles);
      }
    }

    for(size_t i=0;i<collisionLinkPairs_.size();i++){
      std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair = collisionLinkPairs_[i];
      std::shared_ptr<cnoidbodyutils::VclipLinkPair> vclipLinkPair = vclipLinkPairs_[i];

      pcl::PointCloud<pcl::PointXYZ>::Ptr localNearObstacles( new pcl::PointCloud<pcl::PointXYZ>() );
      if(globalNearObstacles->points.size()>0){      // Cannot create a KDTree with an empty input cloud
        pcl::PointXYZ center;
        center.getVector3fMap() = collisionLinkPair->link[0]->p().cast<float>();
        pcl::PointIndices::Ptr localNearIndices(new pcl::PointIndices);
        std::vector<float> distances;
        globalNearObstaclesTreeModel->radiusSearch(center, filterDistanceLocal_, localNearIndices->indices, distances);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(globalNearObstacles);
        extract.setIndices(localNearIndices);
        extract.setNegative(false);
        extract.filter(*localNearObstacles);
      }

      if(allowCollisionBoxFilters_.find(collisionLinkPair->link[0]) != allowCollisionBoxFilters_.end()){
        std::vector<std::shared_ptr<pcl::CropBox<pcl::PointXYZ> > >& boxFilters = allowCollisionBoxFilters_[collisionLinkPair->link[0]];
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered( new pcl::PointCloud<pcl::PointXYZ>() );
        for(size_t i=0;i<boxFilters.size();i++){
          if(localNearObstacles->points.size()==0)break;
          boxFilters[i]->setInputCloud(localNearObstacles);
          boxFilters[i]->setNegative(true);//boxの中を除去する
          boxFilters[i]->filter(*filtered);
          localNearObstacles = filtered;
        }
      }

      cnoid::Vector3 p0_min = vclipLinkPair->link0()->p();
      cnoid::Vector3 p1_min = cnoid::Vector3::Zero();
      double distance_min = 1e10;
      for(size_t j=0;j<localNearObstacles->points.size();j++){
        boxelLink_->p() = localNearObstacles->points[j].getVector3fMap().cast<double>();
        cnoid::Vector3 p0, p1;
        double distance = vclipLinkPair->computeDistance(p0,p1);

        if(distance < distance_min){
          p0_min = p0;
          p1_min = p1;
          distance_min = distance;
        }
      }

      double sign = (distance_min >= 0)? 1.0 : -1.0;
      collisionLinkPair->collisions.resize(2);
      collisionLinkPair->collisions[0].point = p0_min;
      collisionLinkPair->collisions[0].normal = sign * (p1_min - p0_min).normalized();
      collisionLinkPair->collisions[0].depth = - distance_min;
      collisionLinkPair->collisions[1].point = p1_min;
      collisionLinkPair->collisions[1].normal = sign * (p0_min - p1_min).normalized();
      collisionLinkPair->collisions[1].depth = - distance_min;

    }

    //  for visualize
    globalNearObstacles_ = globalNearObstacles;
    return true;
  }

  std::vector<cnoid::SgNodePtr> PCLCollisionDetector::getDrawOnObjects(){
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    {
      for(size_t i=0;i<collisionLinkPairs_.size();i++){
        std::vector<cnoid::SgNodePtr> objects = cnoidbodyutils::collisionLinkPairToDrawOnObjects(collisionLinkPairs_[i],1e5);
        std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
      }
    }
    {
      if(!boxelMaterial_){
        boxelMaterial_ = cnoid::SgMaterialPtr(new cnoid::SgMaterial);
        boxelMaterial_->setTransparency(0.5);
        boxelMaterial_->setDiffuseColor(cnoid::Vector3f(0.0,0.0,1.0));
        boxelMaterial_->setEmissiveColor(cnoid::Vector3f(0.0,0.0,1.0));
      }
      if(!boxelShape_ || boxelShapeSize_ || boxelsize_){
        boxelShape_= cnoid::SgShapePtr(new cnoid::SgShape);
        cnoid::MeshGenerator meshGenerator;
        boxelShape_->setMesh(meshGenerator.generateBox(cnoid::Vector3(boxelsize_, boxelsize_, boxelsize_)));
        boxelShapeSize_ = boxelsize_;
        boxelShape_->setMaterial(boxelMaterial_);
      }
      std::vector<cnoid::SgNodePtr> objects;
      for(size_t i=0;i<globalNearObstacles_->points.size();i++){
        cnoid::SgPosTransformPtr transform(new cnoid::SgPosTransform);
        transform->setTranslation(globalNearObstacles_->points[i].getVector3fMap());
        transform->addChild(boxelShape_);
        objects.push_back(transform);
      }
      std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
    }
    return drawOnObjects;
  }
};
