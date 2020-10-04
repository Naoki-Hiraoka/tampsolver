#include "EditCollisionModel.h"
extern "C" {
#include <qhull/qhull_a.h>
}
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
    int numVertices = model->vertices()->size();
    double points[numVertices*3];
    for (int i=0; i<numVertices; i++){
      const cnoid::Vector3f& v = model->vertices()->at(i);
      points[i*3+0] = v[0];
      points[i*3+1] = v[1];
      points[i*3+2] = v[2];
    }
    char flags[250];
    boolT ismalloc = False;
    sprintf(flags,"qhull Qt Tc");
    if (qh_new_qhull (3,numVertices,points,ismalloc,flags,NULL,stderr)) return nullptr;

    qh_triangulate();
    qh_vertexneighbors();


    cnoid::SgMesh* coldetModel(new cnoid::SgMesh);
    coldetModel->setName(collisionshape->name());

    coldetModel->getOrCreateVertices()->resize(qh num_vertices);
    coldetModel->setNumTriangles(qh num_facets);
    int index[numVertices];
    int vertexIndex = 0;
    vertexT *vertex;
    FORALLvertices {
        int p = qh_pointid(vertex->point);
        index[p] = vertexIndex;
        coldetModel->vertices()->at(vertexIndex++) = cnoid::Vector3f(points[p*3+0], points[p*3+1], points[p*3+2]);
    }
    facetT *facet;
    int num = qh num_facets;
    int triangleIndex = 0;
    FORALLfacets {
        int j = 0, p[3];
        setT *vertices = qh_facet3vertex (facet);
        vertexT **vertexp;
        FOREACHvertexreverse12_ (vertices) {
            if (j<3) {
                p[j] = index[qh_pointid(vertex->point)];
            } else {
                fprintf(stderr, "extra vertex %d\n",j);
            }
            j++;
        }
        coldetModel->setTriangle(triangleIndex++, p[0], p[1], p[2]);
    }

    qh_freeqhull(!qh_ALL);
    int curlong, totlong;
    qh_memfreeshort (&curlong, &totlong);
    if (curlong || totlong) {
        fprintf(stderr, "convhulln: did not free %d bytes of long memory (%d pieces)\n", totlong, curlong);
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
