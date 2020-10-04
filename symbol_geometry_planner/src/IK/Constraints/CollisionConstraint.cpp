#include "CollisionConstraint.h"
#include <cnoid/JointPath>
#include <cnoid/EigenUtil>

namespace IK{

  CollisionConstraint::CollisionConstraint(cnoid::Link* _A_link, cnoid::Link* _B_link):
    A_link(_A_link),
    B_link(_B_link),
    tolerance(0.01),
    maxvel(0.1),
    prev_BA(cnoid::Vector3::UnitX())
  {
  }

  //先にcalc_jacobianineqが呼ばれている前提
  Eigen::VectorXd CollisionConstraint::calc_minineq () {
    Eigen::VectorXd ret(1);
    ret << std::min(tolerance - this->current_distance, maxvel);
    return ret;
  }

  //先にcalc_jacobianineqが呼ばれている前提
  Eigen::VectorXd CollisionConstraint::calc_maxineq () {
    Eigen::VectorXd ret(1);
    ret << 1e30;
    return ret;
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> CollisionConstraint::calc_jacobianineq (const std::vector<cnoid::Body*>& bodies) {
    cnoid::Vector3 A_v, B_v;
    this->current_distance = this->detectDistance(A_v,B_v);

    int dim = 0;
    for(size_t i=0; i < bodies.size(); i++){
      dim += 6 + bodies[i]->numJoints();
    }


    //jacobian A-B
    cnoid::Vector3 BA;
    if(std::abs(this->current_distance)>0.001){
      BA = (A_v - B_v).normalized();
      prev_BA = BA;
    }else{
    //干渉が発生していると，正しく離れる方向を指し示さないことが多い
      BA = prev_BA.normalized();
      //(A_link->wc() - B_link->wc()).normalized()
    }

    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(10);//適当

    if(! (A_link->body() == B_link->body())){
      for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
        int sign = i ? -1 : 1;
        cnoid::Link* target_link = i ? B_link : A_link;
        cnoid::Vector3& target_p = i ? B_v : A_v;

        int idx = 0;
        for(size_t b=0;b<bodies.size();b++){
          if(bodies[b] == target_link->body()){
            //root 6dof
            for(size_t j=0;j<3;j++){
              tripletList.push_back(Eigen::Triplet<double>(0,idx+j,sign*BA[j]));
            }
            cnoid::Vector3 dp = target_p - bodies[b]->rootLink()->p();
            Eigen::Matrix<double, 1, 3> BA_minusdphat = BA.transpose() * - cnoid::hat(dp);
            tripletList.push_back(Eigen::Triplet<double>(0,idx+3,sign*BA_minusdphat[0]));
            tripletList.push_back(Eigen::Triplet<double>(0,idx+4,sign*BA_minusdphat[1]));
            tripletList.push_back(Eigen::Triplet<double>(0,idx+5,sign*BA_minusdphat[2]));

            //joints
            cnoid::JointPath path;
            path.setPath(target_link);
            for(size_t j=0;j<path.numJoints();j++){
              int col = idx+6+path.joint(j)->jointId();
              cnoid::Vector3 omega = path.joint(j)->R() * path.joint(j)->a();
              if(!path.isJointDownward(j)) omega = -omega;
              cnoid::Vector3 dp = omega.cross(target_p - path.joint(j)->p());
              tripletList.push_back(Eigen::Triplet<double>(0,col,sign*BA.dot(dp)));
            }
            break;
          }
          idx += 6 + bodies[b]->numJoints();
        }
      }
    }else{// if(! (A_link->body() == B_link->body()))
      int idx = 0;
      for(size_t b=0;b<bodies.size();b++){
        if(bodies[b] == A_link->body()){
          //joints
          cnoid::JointPath path;
          path.setPath(B_link,A_link);

          const cnoid::Vector3& target_p = A_v;

          for(size_t j=0;j<path.numJoints();j++){
            int col = idx+6+path.joint(j)->jointId();
            cnoid::Vector3 omega = path.joint(j)->R() * path.joint(j)->a();
            if(!path.isJointDownward(j)) omega = -omega;
            cnoid::Vector3 dp = omega.cross(target_p - path.joint(j)->p());
            tripletList.push_back(Eigen::Triplet<double>(0,col,BA.dot(dp)));
          }
          break;
        }
        idx += 6 + bodies[b]->numJoints();
      }
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian(1,dim);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

    return jacobian;
  }

  std::vector<cnoid::SgNodePtr> CollisionConstraint::getDrawOnObjects(){
    if(!this->lines){
      lines = new cnoid::SgLineSet;
      lines->setLineWidth(1.0);
      lines->getOrCreateColors()->resize(2);
      lines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.3,0.0,0.0);
      lines->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
      // A, B
      lines->getOrCreateVertices()->resize(2);
      lines->colorIndices().resize(0);
      lines->addLine(0,1); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
    }

    cnoid::Vector3 A_v, B_v;
    double d = this->detectDistance(A_v,B_v);

    lines->getOrCreateVertices()->at(0) = A_v.cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(1) = B_v.cast<cnoid::Vector3f::Scalar>();
    if (d < tolerance) {
      lines->setLineWidth(3.0);
      lines->colorIndices().at(0) = 1;
      lines->colorIndices().at(1) = 1;
    } else {
      lines->setLineWidth(1.0);
      lines->colorIndices().at(0) = 0;
      lines->colorIndices().at(1) = 0;
    }

    return std::vector<cnoid::SgNodePtr>{lines};
  }

}
