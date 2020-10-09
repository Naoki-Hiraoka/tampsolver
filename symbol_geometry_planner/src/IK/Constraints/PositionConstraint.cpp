#include "PositionConstraint.h"

namespace IK{
  PositionConstraint::PositionConstraint(cnoid::Link* _A_link, const cnoid::Position& _A_localpos,
                                         cnoid::Link* _B_link, const cnoid::Position& _B_localpos):
    A_link(_A_link),
    A_localpos(_A_localpos),
    B_link(_B_link),
    B_localpos(_B_localpos),
    maxvel(0.1)
  {
    return;
  }

  PositionConstraint::PositionConstraint(const cnoid::Position& A_pos,
                                         const cnoid::Position& B_pos):
    PositionConstraint(nullptr,A_pos,nullptr,B_pos)
  {
  }
  PositionConstraint::PositionConstraint(const cnoid::Position& A_pos,
                                         cnoid::Link* B_link, const cnoid::Position& B_localpos):
    PositionConstraint(nullptr,A_pos,B_link,B_localpos)
  {
  }
  PositionConstraint::PositionConstraint(cnoid::Link* A_link, const cnoid::Position& A_localpos,
                                         const cnoid::Position& B_pos):
    PositionConstraint(B_link,B_localpos,nullptr,B_pos)
  {
  }

  PositionConstraint::PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                                         cnoid::Link* B_link, const cnoid::Position& B_localpos):
    PositionConstraint(A->getlink(),A->getlocalpos(),B_link,B_localpos)
  {
  }

  PositionConstraint::PositionConstraint(cnoid::Link* A_link, const cnoid::Position& A_localpos,
                                         std::shared_ptr<RobotConfig::EndEffector> B):
    PositionConstraint(A_link,A_localpos,B->getlink(),B->getlocalpos())
  {
  }

  PositionConstraint::PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                                         std::shared_ptr<RobotConfig::EndEffector> B):
    PositionConstraint(A->getlink(),A->getlocalpos(),B->getlink(),B->getlocalpos())
  {
  }

  PositionConstraint::PositionConstraint(std::shared_ptr<RobotConfig::EndEffector> A,
                                         const cnoid::Position& B_pos):
    PositionConstraint(A->getlink(),A->getlocalpos(),nullptr,B_pos)
  {
  }
  PositionConstraint::PositionConstraint(const cnoid::Position& A_pos,
                                         std::shared_ptr<RobotConfig::EndEffector> B):
    PositionConstraint(nullptr,A_pos,B->getlink(),B->getlocalpos())
  {
  }

  // エラーを返す. A-B. world系. QPで用いる
  const Eigen::VectorXd& PositionConstraint::calc_error () {
    if(this->error.rows()!=6) this->error = Eigen::VectorXd(6);

    const cnoid::Position& A_pos = (this->A_link) ? this->A_link->T() * this->A_localpos : this->A_localpos;
    const cnoid::Position& B_pos = (this->B_link) ? this->B_link->T() * this->B_localpos : this->B_localpos;

    this->error.segment<3>(0) = A_pos.translation() - B_pos.translation();
    this->error.segment<3>(3) = cnoid::omegaFromRot(A_pos.linear() * B_pos.linear().transpose());

    for(size_t i=0;i<3;i++){
      this->error[i] = std::min(std::max(this->error[i],-maxvel),maxvel);
      this->error[i+3] = std::min(std::max(this->error[i+3],-maxvel),maxvel);
    }

    if(debuglevel>1){
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation() << std::endl;
      std::cerr << B_pos.linear() << std::endl;

    }

    return this->error;
  }

  // ヤコビアンを返す. bodyのroot6dof+全関節が変数
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& PositionConstraint::calc_jacobian (const std::vector<cnoid::Body*>& bodies) {
    if(!this->is_bodies_same(bodies,this->jacobian_bodies) || this->jacobian.rows()!=6){
      this->jacobian_bodies = bodies;

      std::vector<Eigen::Triplet<double> > tripletList;
      tripletList.reserve(100);//適当

      if(!A_link || !B_link || !(A_link->body() == B_link->body())){
        for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
          cnoid::Link* target_link = i ? B_link : A_link;
          cnoid::JointPath& path = i? this->path_B : this->path_A;

          if(!target_link) continue;//world固定なので飛ばす

          int idx = 0;
          for(size_t b=0;b<bodies.size();b++){
            if(bodies[b] == target_link->body()){
              //root 6dof
              for(size_t j=0;j<6;j++){
                tripletList.push_back(Eigen::Triplet<double>(j,idx+j,1));
              }
              //  0     p[2] -p[1]
              // -p[2]  0     p[0]
              //  p[1] -p[0]  0
              tripletList.push_back(Eigen::Triplet<double>(0,idx+4,1));
              tripletList.push_back(Eigen::Triplet<double>(0,idx+5,1));
              tripletList.push_back(Eigen::Triplet<double>(1,idx+3,1));
              tripletList.push_back(Eigen::Triplet<double>(1,idx+5,1));
              tripletList.push_back(Eigen::Triplet<double>(2,idx+3,1));
              tripletList.push_back(Eigen::Triplet<double>(2,idx+4,1));

              //joints
              path.setPath(target_link);
              for(size_t j=0;j<path.numJoints();j++){
                int col = idx+6+path.joint(j)->jointId();
                tripletList.push_back(Eigen::Triplet<double>(0,col,1));
                tripletList.push_back(Eigen::Triplet<double>(1,col,1));
                tripletList.push_back(Eigen::Triplet<double>(2,col,1));
                tripletList.push_back(Eigen::Triplet<double>(3,col,1));
                tripletList.push_back(Eigen::Triplet<double>(4,col,1));
                tripletList.push_back(Eigen::Triplet<double>(5,col,1));
              }

              break;
            }
            idx += 6 + bodies[b]->numJoints();
          }
        }
      } else { //if(!A_link || !B_link || !(A_link->body() == B_link->body()))
        int idx = 0;
        for(size_t b=0;b<bodies.size();b++){
          if(bodies[b] == A_link->body()){
            //joints
            this->path_BA.setPath(B_link,A_link);

            for(size_t j=0;j<this->path_BA.numJoints();j++){
              int col = idx+6+this->path_BA.joint(j)->jointId();
              tripletList.push_back(Eigen::Triplet<double>(0,col,1));
              tripletList.push_back(Eigen::Triplet<double>(1,col,1));
              tripletList.push_back(Eigen::Triplet<double>(2,col,1));
              tripletList.push_back(Eigen::Triplet<double>(3,col,1));
              tripletList.push_back(Eigen::Triplet<double>(4,col,1));
              tripletList.push_back(Eigen::Triplet<double>(5,col,1));
            }
            break;
          }
          idx += 6 + bodies[b]->numJoints();
        }
      }

      int dim = 0;
      for(size_t i=0; i < bodies.size(); i++){
        dim += 6 + bodies[i]->numJoints();
      }

      this->jacobian = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,dim);
      this->jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

    }

    if(!A_link || !B_link || !(A_link->body() == B_link->body())){
      for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
        int sign = i ? -1 : 1;
        cnoid::Link* target_link = i ? B_link : A_link;
        const cnoid::Position& target_localpos = i ? B_localpos : A_localpos;
        cnoid::JointPath& path = i ? this->path_B : this->path_A;

        if(!target_link) continue;//world固定なので飛ばす

        const cnoid::Position target_position = target_link->T() * target_localpos;
        const cnoid::Vector3 target_p = target_position.translation();

        int idx = 0;
        for(size_t b=0;b<bodies.size();b++){
          if(bodies[b] == target_link->body()){
            //root 6dof
            for(size_t j=0;j<6;j++){
              this->jacobian.coeffRef(j,idx+j) = sign;
            }
            cnoid::Vector3 dp = target_p - bodies[b]->rootLink()->p();
            //  0     p[2] -p[1]
            // -p[2]  0     p[0]
            //  p[1] -p[0]  0
            this->jacobian.coeffRef(0,idx+4)=sign*dp[2];
            this->jacobian.coeffRef(0,idx+5)=-sign*dp[1];
            this->jacobian.coeffRef(1,idx+3)=-sign*dp[2];
            this->jacobian.coeffRef(1,idx+5)=sign*dp[0];
            this->jacobian.coeffRef(2,idx+3)=sign*dp[1];
            this->jacobian.coeffRef(2,idx+4)=-sign*dp[0];

            //joints
            for(size_t j=0;j<path.numJoints();j++){
              int col = idx+6+path.joint(j)->jointId();
              cnoid::Vector3 omega = path.joint(j)->R() * path.joint(j)->a();
              if(!path.isJointDownward(j)) omega = -omega;
              cnoid::Vector3 dp = omega.cross(target_p - path.joint(j)->p());
              this->jacobian.coeffRef(0,col)=sign*dp[0];
              this->jacobian.coeffRef(1,col)=sign*dp[1];
              this->jacobian.coeffRef(2,col)=sign*dp[2];
              this->jacobian.coeffRef(3,col)=sign*omega[0];
              this->jacobian.coeffRef(4,col)=sign*omega[1];
              this->jacobian.coeffRef(5,col)=sign*omega[2];
            }

            break;
          }
          idx += 6 + bodies[b]->numJoints();
        }
      }
    } else { //if(!A_link || !B_link || !(A_link->body() == B_link->body()))
      int idx = 0;
      for(size_t b=0;b<bodies.size();b++){
        if(bodies[b] == A_link->body()){
          //joints
          const cnoid::Vector3& target_p = A_link->T() * A_localpos.translation();

          for(size_t j=0;j<this->path_BA.numJoints();j++){
            int col = idx+6+this->path_BA.joint(j)->jointId();
            cnoid::Vector3 omega = this->path_BA.joint(j)->R() * this->path_BA.joint(j)->a();
            if(!this->path_BA.isJointDownward(j)) omega = -omega;
            cnoid::Vector3 dp = omega.cross(target_p - this->path_BA.joint(j)->p());
            this->jacobian.coeffRef(0,col)=dp[0];
            this->jacobian.coeffRef(1,col)=dp[1];
            this->jacobian.coeffRef(2,col)=dp[2];
            this->jacobian.coeffRef(3,col)=omega[0];
            this->jacobian.coeffRef(4,col)=omega[1];
            this->jacobian.coeffRef(5,col)=omega[2];
          }
          break;
        }
        idx += 6 + bodies[b]->numJoints();
      }
    }

    return this->jacobian;
  }

  std::vector<cnoid::SgNodePtr> PositionConstraint::getDrawOnObjects(){
    if(!this->lines){
      lines = new cnoid::SgLineSet;
      lines->setLineWidth(1.0);
      lines->getOrCreateColors()->resize(4);
      lines->getOrCreateColors()->at(0) = cnoid::Vector3f(1.0,1.0,1.0);
      lines->getOrCreateColors()->at(1) = cnoid::Vector3f(1.0,0.0,0.0);
      lines->getOrCreateColors()->at(2) = cnoid::Vector3f(0.0,1.0,0.0);
      lines->getOrCreateColors()->at(3) = cnoid::Vector3f(0.0,0.0,1.0);
      // A, A_x, A_y, A_z, B, B_x, B_y, B_z
      lines->getOrCreateVertices()->resize(8);
      lines->colorIndices().resize(0);
      lines->addLine(0,1); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
      lines->addLine(0,2); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
      lines->addLine(0,3); lines->colorIndices().push_back(3); lines->colorIndices().push_back(3);
      lines->addLine(4,5); lines->colorIndices().push_back(1); lines->colorIndices().push_back(1);
      lines->addLine(4,6); lines->colorIndices().push_back(2); lines->colorIndices().push_back(2);
      lines->addLine(4,7); lines->colorIndices().push_back(3); lines->colorIndices().push_back(3);
      lines->addLine(0,4); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
    }

    const cnoid::Position& A_pos = (this->A_link) ? this->A_link->T() * this->A_localpos : this->A_localpos;
    const cnoid::Position& B_pos = (this->B_link) ? this->B_link->T() * this->B_localpos : this->B_localpos;

    lines->getOrCreateVertices()->at(0) = A_pos.translation().cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(1) = (A_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(2) = (A_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(3) = (A_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(4) = B_pos.translation().cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(5) = (B_pos * (0.05 * cnoid::Vector3::UnitX())).cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(6) = (B_pos * (0.05 * cnoid::Vector3::UnitY())).cast<cnoid::Vector3f::Scalar>();
    lines->getOrCreateVertices()->at(7) = (B_pos * (0.05 * cnoid::Vector3::UnitZ())).cast<cnoid::Vector3f::Scalar>();

    return std::vector<cnoid::SgNodePtr>{lines};
  }
}
