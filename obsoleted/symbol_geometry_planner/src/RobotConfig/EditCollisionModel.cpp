#include "EditCollisionModel.h"
#include <qhulleigen/qhulleigen.h>
#include <cnoid/MeshExtractor>
#include <iostream>

namespace RobotConfig {
  cnoid::MeshExtractor* meshExtractor;

  void addMesh(cnoid::SgMesh* model)
  {
    cnoid::SgMesh* mesh = meshExtractor->currentMesh();
    const cnoid::Affine3& T = meshExtractor->currentTransform();

    const int vertexIndexTop = model->getOrCreateVertices()->size();

    const cnoid::SgVertexArray& vertices = *mesh->vertices();
    const int numVertices = vertices.size();
    for(int i=0; i < numVertices; ++i){
      const cnoid::Vector3 v = T * vertices[i].cast<cnoid::Affine3::Scalar>();
      model->vertices()->push_back(v.cast<cnoid::Vector3f::Scalar>());
    }

    const int numTriangles = mesh->numTriangles();
    for(int i=0; i < numTriangles; ++i){
      cnoid::SgMesh::TriangleRef tri = mesh->triangle(i);
      const int v0 = vertexIndexTop + tri[0];
      const int v1 = vertexIndexTop + tri[1];
      const int v2 = vertexIndexTop + tri[2];
      model->addTriangle(v0, v1, v2);
    }
  }

  cnoid::SgMesh* convertToSgMesh (cnoid::SgNode* collisionshape){
    if(!meshExtractor) meshExtractor = new cnoid::MeshExtractor;

    if (!collisionshape) return nullptr;

    cnoid::SgMesh* model = new cnoid::SgMesh;
    if(meshExtractor->extract(collisionshape, [&]() { addMesh(model); })){
      model->setName(collisionshape->name());
    }else{
      std::cerr << "[convertToSgMesh] meshExtractor->extract failed " << collisionshape->name() << std::endl;
      return nullptr;
    }

    return model;
  }


  cnoid::SgNode* convertToConvexHull (cnoid::SgNode* collisionshape){
    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    if (model->vertices()->size()==0) return nullptr;

    // qhull
    Eigen::MatrixXd vertices(3,model->vertices()->size());
    for(size_t i=0;i<model->vertices()->size();i++){
      vertices.col(i) = model->vertices()->at(i).cast<Eigen::Vector3d::Scalar>();
    }
    Eigen::MatrixXd hull;
    std::vector<std::vector<int> > faces;
    if(!qhulleigen::convexhull(vertices,hull,faces)) return nullptr;

    cnoid::SgMesh* coldetModel(new cnoid::SgMesh);
    coldetModel->setName(collisionshape->name());

    coldetModel->getOrCreateVertices()->resize(hull.cols());
    coldetModel->setNumTriangles(faces.size());

    for(size_t i=0;i<hull.cols();i++){
      coldetModel->vertices()->at(i) = hull.col(i).cast<cnoid::Vector3f::Scalar>();
    }

    for(size_t i=0;i<faces.size();i++){
      coldetModel->setTriangle(i, faces[i][0], faces[i][1], faces[i][2]);
    }

    cnoid::SgShape* ret(new cnoid::SgShape);
    ret->setMesh(coldetModel);
    ret->setName(collisionshape->name());
    return ret;
  }

  Vclip::Polyhedron* convertToVClipModel (cnoid::SgNode* collisionshape){
    cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

    Vclip::Polyhedron* i_vclip_model = new Vclip::Polyhedron();
    int n = model->vertices()->size();
    Vclip::VertFaceName vertName;
    for (int i = 0; i < n; i ++ ) {
      const cnoid::Vector3f& v = model->vertices()->at(i);
      sprintf(vertName, "v%d", i);
      i_vclip_model->addVertex(vertName, Vclip::Vect3(v[0], v[1], v[2]));
    }
    i_vclip_model->buildHull();
    i_vclip_model->check();
    fprintf(stderr, "[Vclip] build finished, vcliip mesh of %s, %d -> %d\n",
            collisionshape->name().c_str(), n, (int)(i_vclip_model->verts().size()));
    return i_vclip_model;
  }
}
