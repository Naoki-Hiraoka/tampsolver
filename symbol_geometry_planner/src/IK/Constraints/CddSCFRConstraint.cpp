#include "CddSCFRConstraint.h"
#include <cddeigen/cddeigen.h>

extern "C" {
#include <qhull/qhull_a.h>
}

namespace IK{
  CddSCFRConstraint::CddSCFRConstraint(cnoid::Body* robot, const std::vector<std::shared_ptr<RobotConfig::EndEffector> >& endeffectors):
    SCFRConstraint(robot,endeffectors)
  {
  }

  // Ax = b, Cx >= d
  void CddSCFRConstraint::calcProjection(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d){

    /*
      INPUT:
        A_eq   x + b_eq    = 0
        A_ineq x + b_ineq >= 0
      OUTPUT:
        x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
    */
    Eigen::MatrixXd V, R_nonneg, R_free;
    cddeigen::HtoVgmp(A,-b,C,-d,V,R_nonneg,R_free,this->debuglevel);

    // get p_x, p_y component
    this->V2 = V.topRows<2>();
    this->R_nonneg2 = R_nonneg.topRows<2>();
    this->R_free2 = R_free.topRows<2>();

    /*
      INPUT:
        x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
      OUTPUT:
        A_eq   x + b_eq    = 0
        A_ineq x + b_ineq >= 0
    */
    Eigen::MatrixXd A2, C2;
    Eigen::VectorXd b2, d2;

    cddeigen::VtoHgmp(this->V2,this->R_nonneg2,this->R_free2,A2,b2,C2,d2,this->debuglevel);

    // create SCFR
    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(10);//適当
    for(size_t i=0;i<A2.rows();i++){
      for(size_t j=0;j<A2.cols();j++){
        if(A2(i,j)!=0) tripletList.push_back(Eigen::Triplet<double>(i,j,A2(i,j)));
      }
    }
    for(size_t i=0;i<C2.rows();i++){
      for(size_t j=0;j<C2.cols();j++){
        if(C2(i,j)!=0) tripletList.push_back(Eigen::Triplet<double>(A2.rows()+i,j,C2(i,j)));
      }
    }
    this->SCFR_M = Eigen::SparseMatrix<double,Eigen::RowMajor>(A2.rows()+C2.rows(),2);
    this->SCFR_M.setFromTriplets(tripletList.begin(), tripletList.end());
    this->SCFR_u = Eigen::VectorXd(b2.rows()+d2.rows());
    this->SCFR_u.head(b2.rows()) = -b2;
    for(size_t i=b2.rows();i<this->SCFR_u.rows();i++) this->SCFR_u[i] = 1e30;
    this->SCFR_l = Eigen::VectorXd(b2.rows()+d2.rows());
    this->SCFR_l.head(b2.rows()) = -b2;
    this->SCFR_l.tail(d2.rows()) = -d2;

    return;
  }

  void CddSCFRConstraint::updateSCFRlines(){
    if(this->initial_p){
      this->calcSCFR();
      this->initial_p = false;
    }

    if(!this->SCFRlines){
      this->SCFRlines = new cnoid::SgLineSet;
      this->SCFRlines->setLineWidth(3.0);
      this->SCFRlines->getOrCreateColors()->resize(1);
      this->SCFRlines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,1.0,0.0);

      this->SCFRlines->getOrCreateVertices()->resize(0);
      this->SCFRlines->colorIndices().resize(0);

      // calculate V hull
      std::vector<cnoid::Vector2> V_hull;
      std::vector<std::vector<int> > hull_index;
      V_hull.reserve(10);//適当
      hull_index.reserve(10);//適当
      if(V2.cols() <= 2){
        for(size_t i=0;i<V2.cols();i++){
          V_hull.push_back(cnoid::Vector2(V2(0,i),V2(1,i)));
        }
      }else{
        // convex hull by qhull.
        int numVertices = this->V2.cols();
        double points[numVertices*2];
        for (int i=0; i<numVertices; i++){
          points[i*2+0] = V2(0,i);
          points[i*2+1] = V2(1,i);
        }
        char flags[250];
        boolT ismalloc = False;
        sprintf(flags,"qhull Fx");
        if (qh_new_qhull (2,numVertices,points,ismalloc,flags,NULL,stderr)) return;

        qh_triangulate();
        qh_vertexneighbors();

        int index[numVertices];
        int vertexIndex = 0;
        vertexT *vertex;
        FORALLvertices {
          int p = qh_pointid(vertex->point);
          index[p] = vertexIndex;//なぜか分からないがpの値が数百あるので，多分ここでメモリ確保違反をしている
          V_hull.push_back(cnoid::Vector2(V2(0,p),V2(1,p)));
          vertexIndex++;
        }

        facetT *facet;
        int num = qh num_facets;
        int triangleIndex = 0;
        FORALLfacets {
          int j = 0, p[2];
          setT *vertices = facet->vertices;
          //setT *vertices = qh_facet3vertex (facet);
          vertexT **vertexp;
          FOREACHvertexreverse12_ (vertices) {
            if (j<2) {
              p[j] = index[qh_pointid(vertex->point)];
            } else {
              fprintf(stderr, "extra vertex %d\n",j);
            }
            j++;
          }
          hull_index.push_back(std::vector<int>{p[0],p[1]});

        }

        qh_freeqhull(!qh_ALL);
        int curlong, totlong;
        qh_memfreeshort (&curlong, &totlong);

      }

      this->SCFRlines->getOrCreateVertices()->resize(V_hull.size());
      for(size_t i=0;i<V_hull.size();i++){
        this->SCFRlines->vertices()->at(i) = cnoid::Vector3f(V_hull[i][0],V_hull[i][1],0);
      }
      for(size_t i=0;i<hull_index.size();i++){
        this->SCFRlines->addLine(hull_index[i][0],hull_index[i][1]);
        this->SCFRlines->colorIndices().push_back(0);
        this->SCFRlines->colorIndices().push_back(0);
      }
    }

    // z座標は毎回更新する
    double z = this->robot->calcCenterOfMass()[2];
    for(size_t i=0;i<this->SCFRlines->vertices()->size();i++){
      this->SCFRlines->vertices()->at(i)[2] = z;
    }
  }

}
