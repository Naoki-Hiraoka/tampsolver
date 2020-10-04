#include "PositionConstraint.h"
#include <cnoid/JointPath>

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
  Eigen::VectorXd PositionConstraint::calc_error () {
    const cnoid::Position& A_pos = (this->A_link) ? this->A_link->T() * this->A_localpos : this->A_localpos;
    const cnoid::Position& B_pos = (this->B_link) ? this->B_link->T() * this->B_localpos : this->B_localpos;

    Eigen::VectorXd error(6);
    error.segment<3>(0) = A_pos.translation() - B_pos.translation();
    error.segment<3>(3) = cnoid::omegaFromRot(A_pos.linear() * B_pos.linear().transpose());

    for(size_t i=0;i<3;i++){
      error[i] = std::min(std::max(error[i],-maxvel),maxvel);
      error[i+3] = std::min(std::max(error[i+3],-maxvel),maxvel);
    }

    if(debuglevel>1){
      std::cerr << "A_pos" << std::endl;
      std::cerr << A_pos.translation() << std::endl;
      std::cerr << A_pos.linear() << std::endl;
      std::cerr << "B_pos" << std::endl;
      std::cerr << B_pos.translation() << std::endl;
      std::cerr << B_pos.linear() << std::endl;

    }

    return error;
  }

  // ヤコビアンを返す. bodyのroot6dof+全関節が変数
  Eigen::SparseMatrix<double,Eigen::RowMajor> PositionConstraint::calc_jacobian (const std::vector<cnoid::Body*>& bodies) {
    int dim = 0;
    for(size_t i=0; i < bodies.size(); i++){
      dim += 6 + bodies[i]->numJoints();
    }


    std::vector<Eigen::Triplet<double> > tripletList;
    tripletList.reserve(100);//適当

    if(!A_link || !B_link || !(A_link->body() == B_link->body())){
      for(size_t i=0;i<2;i++){//0:A_link, 1:B_link
        int sign = i ? -1 : 1;
        cnoid::Link* target_link = i ? B_link : A_link;
        const cnoid::Position& target_localpos = i ? B_localpos : A_localpos;

        if(!target_link) continue;//world固定なので飛ばす

        const cnoid::Position target_position = target_link->T() * target_localpos;
        const cnoid::Vector3 target_p = target_position.translation();

        int idx = 0;
        for(size_t b=0;b<bodies.size();b++){
          if(bodies[b] == target_link->body()){
            //root 6dof
            for(size_t j=0;j<6;j++){
              tripletList.push_back(Eigen::Triplet<double>(j,idx+j,sign));
            }
            cnoid::Vector3 dp = target_p - bodies[b]->rootLink()->p();
            //  0     p[2] -p[1]
            // -p[2]  0     p[0]
            //  p[1] -p[0]  0
            tripletList.push_back(Eigen::Triplet<double>(0,idx+4,sign*dp[2]));
            tripletList.push_back(Eigen::Triplet<double>(0,idx+5,-sign*dp[1]));
            tripletList.push_back(Eigen::Triplet<double>(1,idx+3,-sign*dp[2]));
            tripletList.push_back(Eigen::Triplet<double>(1,idx+5,sign*dp[0]));
            tripletList.push_back(Eigen::Triplet<double>(2,idx+3,sign*dp[1]));
            tripletList.push_back(Eigen::Triplet<double>(2,idx+4,-sign*dp[0]));

            //joints
            cnoid::JointPath path;
            path.setPath(target_link);
            for(size_t j=0;j<path.numJoints();j++){
              int col = idx+6+path.joint(j)->jointId();
              cnoid::Vector3 omega = path.joint(j)->R() * path.joint(j)->a();
              if(!path.isJointDownward(j)) omega = -omega;
              cnoid::Vector3 dp = omega.cross(target_p - path.joint(j)->p());
              tripletList.push_back(Eigen::Triplet<double>(0,col,sign*dp[0]));
              tripletList.push_back(Eigen::Triplet<double>(1,col,sign*dp[1]));
              tripletList.push_back(Eigen::Triplet<double>(2,col,sign*dp[2]));
              tripletList.push_back(Eigen::Triplet<double>(3,col,sign*omega[0]));
              tripletList.push_back(Eigen::Triplet<double>(4,col,sign*omega[1]));
              tripletList.push_back(Eigen::Triplet<double>(5,col,sign*omega[2]));
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
          cnoid::JointPath path;
          path.setPath(B_link,A_link);

          const cnoid::Vector3& target_p = A_link->T() * A_localpos.translation();

          for(size_t j=0;j<path.numJoints();j++){
            int col = idx+6+path.joint(j)->jointId();
            cnoid::Vector3 omega = path.joint(j)->R() * path.joint(j)->a();
            if(!path.isJointDownward(j)) omega = -omega;
            cnoid::Vector3 dp = omega.cross(target_p - path.joint(j)->p());
            tripletList.push_back(Eigen::Triplet<double>(0,col,dp[0]));
            tripletList.push_back(Eigen::Triplet<double>(1,col,dp[1]));
            tripletList.push_back(Eigen::Triplet<double>(2,col,dp[2]));
            tripletList.push_back(Eigen::Triplet<double>(3,col,omega[0]));
            tripletList.push_back(Eigen::Triplet<double>(4,col,omega[1]));
            tripletList.push_back(Eigen::Triplet<double>(5,col,omega[2]));
          }
          break;
        }
        idx += 6 + bodies[b]->numJoints();
      }
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian(6,dim);
    jacobian.setFromTriplets(tripletList.begin(), tripletList.end());

    return jacobian;
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
