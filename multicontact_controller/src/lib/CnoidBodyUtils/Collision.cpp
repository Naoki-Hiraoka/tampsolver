#include <multicontact_controller/lib/CnoidBodyUtils/Collision.h>

#include <vector>
#include <vclip.h> // 先にvectorをincludeする必要あり。using namespace stdしてしまうことに注意

#include <qhulleigen/qhulleigen.h>
#include <cnoid/MeshExtractor>
#include <cnoid/EigenUtil>

#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    void addMesh(cnoid::SgMesh* model, std::shared_ptr<cnoid::MeshExtractor> meshExtractor){
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

      if (!collisionshape) return nullptr;

      std::shared_ptr<cnoid::MeshExtractor> meshExtractor = std::make_shared<cnoid::MeshExtractor>();
      cnoid::SgMesh* model = new cnoid::SgMesh;
      if(meshExtractor->extract(collisionshape, [&]() { addMesh(model,meshExtractor); })){
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

    void convertCollisionToConvexHull(cnoid::Body* robot){
      for(size_t i=0;i<robot->numLinks();i++){
        cnoid::SgNode* coldetModel = convertToConvexHull(robot->link(i)->collisionShape());
        if(coldetModel){
          robot->link(i)->setCollisionShape(coldetModel);
        }else{
          std::cerr << "[convertCollisionToConvexHull] convex hull " << robot->link(i)->name() << " fail" << std::endl;
        }
      }
    }

    std::shared_ptr<Vclip::Polyhedron> convertToVClipModel (cnoid::SgNode* collisionshape){
      cnoid::SgMeshPtr model = convertToSgMesh(collisionshape);

      std::shared_ptr<Vclip::Polyhedron> i_vclip_model = std::make_shared<Vclip::Polyhedron>();
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

    VclipLinkPair::VclipLinkPair(const cnoid::Link* link0, std::shared_ptr<Vclip::Polyhedron> vclip_model0, const cnoid::Link* link1, std::shared_ptr<Vclip::Polyhedron> vclip_model1, double tolerance)
      : link1_(link0),
        link2_(link1),
        Vclip_Model1(vclip_model0),
        Vclip_Model2(vclip_model1),
        tolerance_(tolerance),
        q1_(cnoid::Vector3::Zero()),
        q2_(cnoid::Vector3::Zero())
    {
      Feature_Pair = std::make_shared<Vclip::FeaturePair>();
      // Vclip::VertexはVclip::Featureをprivate継承しているのでshare_ptrだとcastできない
      Feature_Pair->first  = (const Vclip::Feature *)new Vclip::Vertex(Vclip_Model1->verts().front());
      Feature_Pair->second = (const Vclip::Feature *)new Vclip::Vertex(Vclip_Model2->verts().front());
    }

    bool VclipLinkPair::checkCollision()
    {
      cnoid::Vector3 p1, p2;
      double len = computeDistance(p1,p2);
      if ( len < tolerance_ ) {
        return true;
      }
      return false;
    }

    double VclipLinkPair::computeDistance(cnoid::Vector3& q1, cnoid::Vector3& q2)
    {
      Vclip::Mat3 R1, R2;
      Vclip::Vect3 T1, T2;
      Vclip::VclipPose P1, P2;
      const cnoid::Vector3& p1 = link1_->p();
      cnoid::Matrix3 r1 = link1_->attitude();
      const cnoid::Vector3&  p2 = link2_->p();
      cnoid::Matrix3 r2 = link2_->attitude();
      R1.xrow().set(r1(0,0), r1(0,1), r1(0,2));
      R1.yrow().set(r1(1,0), r1(1,1), r1(1,2));
      R1.zrow().set(r1(2,0), r1(2,1), r1(2,2));
      R2.xrow().set(r2(0,0), r2(0,1), r2(0,2));
      R2.yrow().set(r2(1,0), r2(1,1), r2(1,2));
      R2.zrow().set(r2(2,0), r2(2,1), r2(2,2));
      T1.set(p1(0), p1(1), p1(2));
      T2.set(p2(0), p2(1), p2(2));
      P1.set(R1, T1);
      P2.set(R2, T2);
      Vclip::VclipPose X12, X21;
      X12.invert(P2);
      X12.postmult(P1);
      X21.invert(X12);
      Vclip::Vect3 cp1, cp2;
      const Vclip::Polyhedron* Vclip_Model1_raw = Vclip_Model1.get();
      const Vclip::Polyhedron* Vclip_Model2_raw = Vclip_Model2.get();
      double len = Vclip::Polyhedron::vclip(Vclip_Model1_raw, Vclip_Model2_raw, X12, X21, Feature_Pair->first, Feature_Pair->second, cp1, cp2, 0);
      Vclip::Vect3 cp1g, cp2g;
      P1.xformPoint(cp1, cp1g);
      P2.xformPoint(cp2, cp2g);
      q1[0] = cp1g.x; q1[1] = cp1g.y; q1[2] = cp1g.z;
      q2[0] = cp2g.x; q2[1] = cp2g.y; q2[2] = cp2g.z;

      q1_ = q1;
      q2_ = q2;
      distance_ = len;
      return len;
    }

    double VclipLinkPair::computeDistanceLocal(cnoid::Vector3& q1, cnoid::Vector3& q2){
      cnoid::Vector3 q1_world, q2_world;
      double len = computeDistance(q1_world,q2_world);
      q1 = link1_->T().inverse() * q1_world;
      q2 = link2_->T().inverse() * q2_world;
      return len;
    }

    std::vector<cnoid::SgNodePtr> VclipLinkPair::getDrawOnObjects(){
      if(!this->lines){
        this->lines = new cnoid::SgLineSet;
        this->lines->setLineWidth(1.0);
        this->lines->getOrCreateColors()->resize(3);
        this->lines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.3,0.0,0.0);
        this->lines->getOrCreateColors()->at(1) = cnoid::Vector3f(0.6,0.0,0.0);
        this->lines->getOrCreateColors()->at(2) = cnoid::Vector3f(1.0,0.0,0.0);
        // A, B
        this->lines->getOrCreateVertices()->resize(2);
        this->lines->colorIndices().resize(0);
        this->lines->addLine(0,1); this->lines->colorIndices().push_back(0); this->lines->colorIndices().push_back(0);
      }

      cnoid::Vector3 A_v = q1_;
      cnoid::Vector3 B_v = q2_;
      double d = distance_;

      this->lines->vertices()->at(0) = A_v.cast<cnoid::Vector3f::Scalar>();
      this->lines->vertices()->at(1) = B_v.cast<cnoid::Vector3f::Scalar>();
      if (d < tolerance_) {
        this->lines->setLineWidth(3.0);
        this->lines->colorIndices().at(0) = 2;
        this->lines->colorIndices().at(1) = 2;
      } else if (d < tolerance_ * 2) {
        this->lines->setLineWidth(2.0);
        this->lines->colorIndices().at(0) = 1;
        this->lines->colorIndices().at(1) = 1;
      } else {
        this->lines->setLineWidth(1.0);
        this->lines->colorIndices().at(0) = 0;
        this->lines->colorIndices().at(1) = 0;
      }

      return std::vector<cnoid::SgNodePtr>{this->lines};
    }

    Collision::Collision(cnoid::Body* robot)
      : robot_(robot),
        tolerance_(0.02),
        maxDistance_(0.2),
        distance_(0.1)
    {
      collisionLinkPair_.collisions.resize(2);
    }

    void Collision::getCollisionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
      //distanceを更新
      p0_ = collisionLinkPair_.link[0] ? collisionLinkPair_.link[0]->T() * collisionLinkPair_.collisions[0].point : collisionLinkPair_.collisions[0].point;
      p1_ = collisionLinkPair_.link[1] ? collisionLinkPair_.link[1]->T() * collisionLinkPair_.collisions[1].point : collisionLinkPair_.collisions[1].point;
      cnoid::Vector3 normal01 = (collisionLinkPair_.link[0] ? collisionLinkPair_.link[0]->R() * collisionLinkPair_.collisions[0].normal : collisionLinkPair_.collisions[0].normal).normalized();//p1がこの方向に動くと離れる
      if(collisionLinkPair_.collisions[0].depth > -1e5){
        distance_ = (p1_ - p0_).norm() * ((p1_ - p0_).dot(normal01) >= 0 ? 1.0 : -1.0);
      }else{
        //非常に遠い. このときp0, p1にはでたらめな値が入っている場合があるので、distanceは計算しない
        distance_ = - collisionLinkPair_.collisions[0].depth;
      }

      if(distance_ > maxDistance_){
        //考慮しない
        A.resize(0,6+robot_->numJoints());
        b.resize(0);
        wa.resize(0);
        C.resize(0,6+robot_->numJoints());
        dl.resize(0);
        du.resize(0);
        wc.resize(0);
        return;
      }

      A.resize(0,6+robot_->numJoints());
      b.resize(0);
      wa.resize(0);
      C = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,6+robot_->numJoints());
      dl.resize(1);
      dl[0] = - distance_ + tolerance_;
      du.resize(1);
      du[0] = std::numeric_limits<double>::max();
      wc.resize(1);
      wc[0] = 1.0;
      if(collisionLinkPair_.body[0] && collisionLinkPair_.link[0] &&
         collisionLinkPair_.body[1] && collisionLinkPair_.link[1] &&
         collisionLinkPair_.body[0] == collisionLinkPair_.body[1] &&
         collisionLinkPair_.body[0] == robot_){

        if(path_.empty() ||
           path_.baseLink() != collisionLinkPair_.link[0] ||
           path_.endLink() != collisionLinkPair_.link[1]){
          path_.setPath(collisionLinkPair_.link[0], collisionLinkPair_.link[1]);
        }
        for(size_t j=0;j<path_.numJoints();j++){
          int col = 6+path_.joint(j)->jointId();
          cnoid::Vector3 omega = path_.joint(j)->R() * path_.joint(j)->a();
          if(!path_.isJointDownward(j)) omega = -omega;
          cnoid::Vector3 dp = omega.cross(p1_ - path_.joint(j)->p());
          C.insert(0,col)=normal01.dot(dp);
        }
      }else{
        if(collisionLinkPair_.link[0] && collisionLinkPair_.body[0] &&
           collisionLinkPair_.body[0] == robot_){
          if(path0_.empty() ||
             path0_.baseLink() != robot_->rootLink() ||
             path0_.endLink() != collisionLinkPair_.link[0]){
            path0_.setPath(collisionLinkPair_.link[0]);
          }
          //root 6dof
          for(size_t j=0;j<3;j++){
            C.coeffRef(0,j) = -1.0 * normal01[j];
          }
          cnoid::Vector3 dp = p0_ - robot_->rootLink()->p();
          Eigen::Matrix<double, 1, 3> normal01_minusdphat = normal01.transpose() * - cnoid::hat(dp);
          C.coeffRef(0,3)=-1.0*normal01_minusdphat[0];
          C.coeffRef(0,4)=-1.0*normal01_minusdphat[1];
          C.coeffRef(0,5)=-1.0*normal01_minusdphat[2];

          //joints
          for(size_t j=0;j<path0_.numJoints();j++){
            int col = 6+path0_.joint(j)->jointId();
            cnoid::Vector3 omega = path0_.joint(j)->R() * path0_.joint(j)->a();
            if(!path0_.isJointDownward(j)) omega = -omega;
            cnoid::Vector3 dp = omega.cross(p0_ - path0_.joint(j)->p());
            C.coeffRef(0,col)=-1.0*normal01.dot(dp);
          }
        }
        if(collisionLinkPair_.link[1] && collisionLinkPair_.body[1] &&
           collisionLinkPair_.body[1] == robot_){
          if(path1_.empty() ||
             path1_.baseLink() != robot_->rootLink() ||
             path1_.endLink() != collisionLinkPair_.link[1]){
            path1_.setPath(collisionLinkPair_.link[1]);
          }
          //root 6dof
          for(size_t j=0;j<3;j++){
            C.coeffRef(0,j) = +1.0 * normal01[j];
          }
          cnoid::Vector3 dp = p1_ - robot_->rootLink()->p();
          Eigen::Matrix<double, 1, 3> normal01_minusdphat = normal01.transpose() * - cnoid::hat(dp);
          C.coeffRef(0,3)=+1.0*normal01_minusdphat[0];
          C.coeffRef(0,4)=+1.0*normal01_minusdphat[1];
          C.coeffRef(0,5)=+1.0*normal01_minusdphat[2];

          //joints
          for(size_t j=0;j<path1_.numJoints();j++){
            int col = 6+path1_.joint(j)->jointId();
            cnoid::Vector3 omega = path1_.joint(j)->R() * path1_.joint(j)->a();
            if(!path1_.isJointDownward(j)) omega = -omega;
            cnoid::Vector3 dp = omega.cross(p1_ - path1_.joint(j)->p());
            C.coeffRef(0,col)=+1.0*normal01.dot(dp);
          }
        }
      }
    }

    void collisionArrayMsgToCnoid(cnoid::Body* robot, const multicontact_controller_msgs::CollisionArray& msg, std::vector<std::shared_ptr<Collision> >& collisions){
      collisions.resize(msg.collisions.size());
      for(size_t i=0;i<msg.collisions.size();i++){
        if(!collisions[i]) collisions[i] = std::make_shared<Collision>(robot);
        if(msg.collisions[i].point1.header.frame_id == "odom"){
          collisions[i]->collisionLinkPair().body[0] = nullptr;
          collisions[i]->collisionLinkPair().link[0] = nullptr;
        }else if(collisions[i]->collisionLinkPair().link[0] &&
                 msg.collisions[i].point1.header.frame_id == collisions[i]->collisionLinkPair().link[0]->name()){
          //do nothing
        }else if(robot->link(msg.collisions[i].point1.header.frame_id)){
          collisions[i]->collisionLinkPair().body[0] = robot;
          collisions[i]->collisionLinkPair().link[0] = robot->link(msg.collisions[i].point1.header.frame_id);
        }else{
          ROS_ERROR("%s not found", msg.collisions[i].point1.header.frame_id.c_str());
          collisions[i]->collisionLinkPair().body[0] = nullptr;
          collisions[i]->collisionLinkPair().link[0] = nullptr;
        }
        if(msg.collisions[i].point2.header.frame_id == "odom"){
          collisions[i]->collisionLinkPair().body[1] = nullptr;
          collisions[i]->collisionLinkPair().link[1] = nullptr;
        }else if(collisions[i]->collisionLinkPair().link[1] &&
                 msg.collisions[i].point2.header.frame_id == collisions[i]->collisionLinkPair().link[1]->name()){
          //do nothing
        }else if(robot->link(msg.collisions[i].point2.header.frame_id)){
          collisions[i]->collisionLinkPair().body[1] = robot;
          collisions[i]->collisionLinkPair().link[1] = robot->link(msg.collisions[i].point2.header.frame_id);
        }else{
          ROS_ERROR("%s not found", msg.collisions[i].point2.header.frame_id.c_str());
          collisions[i]->collisionLinkPair().body[1] = nullptr;
          collisions[i]->collisionLinkPair().link[1] = nullptr;
        }

        collisions[i]->collisionLinkPair().collisions.resize(2);
        tf::pointMsgToEigen(msg.collisions[i].point1.point, collisions[i]->collisionLinkPair().collisions[0].point);
        tf::pointMsgToEigen(msg.collisions[i].point2.point, collisions[i]->collisionLinkPair().collisions[1].point);
        tf::vectorMsgToEigen(msg.collisions[i].normal1.vector, collisions[i]->collisionLinkPair().collisions[0].normal);
        tf::vectorMsgToEigen(msg.collisions[i].normal2.vector, collisions[i]->collisionLinkPair().collisions[1].normal);
        collisions[i]->collisionLinkPair().collisions[0].depth = - msg.collisions[i].distance;
        collisions[i]->collisionLinkPair().collisions[1].depth = - msg.collisions[i].distance;
      }
    }

    std::vector<cnoid::SgNodePtr> Collision::getDrawOnObjects(){
      if(!this->lines_){
        this->lines_ = new cnoid::SgLineSet;
        this->lines_->setLineWidth(1.0);
        this->lines_->getOrCreateColors()->resize(3);
        {
          if(collisionLinkPair_.link[0] && collisionLinkPair_.link[1]){
            this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.3,0.0,0.0);
            this->lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(0.6,0.0,0.0);
            this->lines_->getOrCreateColors()->at(2) = cnoid::Vector3f(1.0,0.0,0.0);
          }else{
            this->lines_->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,0.0,0.3);
            this->lines_->getOrCreateColors()->at(1) = cnoid::Vector3f(0.0,0.0,0.6);
            this->lines_->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,0.0,1.0);
          }
        }

        // A, B
        this->lines_->getOrCreateVertices()->resize(2);
        this->lines_->colorIndices().resize(0);
        this->lines_->addLine(0,1); this->lines_->colorIndices().push_back(0); this->lines_->colorIndices().push_back(0);
      }

      cnoid::Vector3 A_v = p0_;
      cnoid::Vector3 B_v = p1_;
      double d = distance_;

      if(distance_ > maxDistance_) return std::vector<cnoid::SgNodePtr>();

      this->lines_->vertices()->at(0) = A_v.cast<cnoid::Vector3f::Scalar>();
      this->lines_->vertices()->at(1) = B_v.cast<cnoid::Vector3f::Scalar>();
      if (d < tolerance_) {
        this->lines_->setLineWidth(3.0);
        this->lines_->colorIndices().at(0) = 2;
        this->lines_->colorIndices().at(1) = 2;
      } else if (d < tolerance_ * 2) {
        this->lines_->setLineWidth(2.0);
        this->lines_->colorIndices().at(0) = 1;
        this->lines_->colorIndices().at(1) = 1;
      } else {
        this->lines_->setLineWidth(1.0);
        this->lines_->colorIndices().at(0) = 0;
        this->lines_->colorIndices().at(1) = 0;
      }

      return std::vector<cnoid::SgNodePtr>{this->lines_};
    }

    std::vector<cnoid::SgNodePtr> collisionLinkPairToDrawOnObjects(std::shared_ptr<cnoid::CollisionLinkPair>& collisionLinkPair, double thre){
      std::vector<cnoid::SgNodePtr> objects;
      for(size_t i=0;i<collisionLinkPair->collisions.size()/2;i++){
        cnoid::Vector3 A_v = collisionLinkPair->collisions[i*2+0].point;
        cnoid::Vector3 B_v = collisionLinkPair->collisions[i*2+1].point;
        double d = - collisionLinkPair->collisions[i*2+0].depth;
        if(d>thre)continue;

        cnoid::SgLineSetPtr lines(new cnoid::SgLineSet);
        lines->setLineWidth(1.0);
        lines->getOrCreateColors()->resize(1);
        {
          cnoid::Vector3f color;
          if(collisionLinkPair->link[0] && collisionLinkPair->link[1]){
            color = cnoid::Vector3f(0.3,0.0,0.0);
          }else{
            color = cnoid::Vector3f(0.0,0.0,0.3);
          }
          lines->getOrCreateColors()->at(0) = color;
        }
        // A, B
        lines->getOrCreateVertices()->resize(2);
        lines->colorIndices().resize(0);
        lines->addLine(0,1); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);

        lines->vertices()->at(0) = A_v.cast<cnoid::Vector3f::Scalar>();
        lines->vertices()->at(1) = B_v.cast<cnoid::Vector3f::Scalar>();
        lines->setLineWidth(1.0);
        lines->colorIndices().at(0) = 0;
        lines->colorIndices().at(1) = 0;

        objects.push_back(lines);
      }

      return objects;
    }

  };
};

