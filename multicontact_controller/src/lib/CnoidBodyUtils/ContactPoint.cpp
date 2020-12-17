#include <multicontact_controller/lib/CnoidBodyUtils/ContactPoint.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPoint::calcJacobian(cnoid::Link* parent, cnoid::Position& T_local){//world系,contactpoint周り
      if(path_.empty() || path_.endLink() != parent){

        std::vector<Eigen::Triplet<double> > tripletList;
        tripletList.reserve(100);//適当

        //root 6dof
        for(size_t j=0;j<6;j++){
          tripletList.push_back(Eigen::Triplet<double>(j,j,1));
        }
        //  0     p[2] -p[1]
        // -p[2]  0     p[0]
        //  p[1] -p[0]  0
        tripletList.push_back(Eigen::Triplet<double>(0,4,1));
        tripletList.push_back(Eigen::Triplet<double>(0,5,1));
        tripletList.push_back(Eigen::Triplet<double>(1,3,1));
        tripletList.push_back(Eigen::Triplet<double>(1,5,1));
        tripletList.push_back(Eigen::Triplet<double>(2,3,1));
        tripletList.push_back(Eigen::Triplet<double>(2,4,1));

        //joints
        path_.setPath(parent);
        for(size_t j=0;j<path_.numJoints();j++){
          int col = 6+path_.joint(j)->jointId();
          tripletList.push_back(Eigen::Triplet<double>(0,col,1));
          tripletList.push_back(Eigen::Triplet<double>(1,col,1));
          tripletList.push_back(Eigen::Triplet<double>(2,col,1));
          tripletList.push_back(Eigen::Triplet<double>(3,col,1));
          tripletList.push_back(Eigen::Triplet<double>(4,col,1));
          tripletList.push_back(Eigen::Triplet<double>(5,col,1));
        }

        jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6+parent->body()->numJoints());
        jacobian_.setFromTriplets(tripletList.begin(), tripletList.end());
      }

      const cnoid::Position target_position = parent->T() * T_local;
      const cnoid::Vector3 target_p = target_position.translation();
      //root 6dof
      for(size_t j=0;j<6;j++){
        this->jacobian_.coeffRef(j,j) = 1;
      }
      cnoid::Vector3 dp = target_p - parent->body()->rootLink()->p();
      //  0     p[2] -p[1]
      // -p[2]  0     p[0]
      //  p[1] -p[0]  0
      jacobian_.coeffRef(0,4)=dp[2];
      jacobian_.coeffRef(0,5)=-dp[1];
      jacobian_.coeffRef(1,3)=-dp[2];
      jacobian_.coeffRef(1,5)=dp[0];
      jacobian_.coeffRef(2,3)=dp[1];
      jacobian_.coeffRef(2,4)=-dp[0];

      //joints
      for(size_t j=0;j<path_.numJoints();j++){
        int col = 6+path_.joint(j)->jointId();
        cnoid::Vector3 omega = path_.joint(j)->R() * path_.joint(j)->a();
        if(!path_.isJointDownward(j)) omega = -omega;
        if(path_.joint(j)->jointType() == cnoid::Link::JointType::PRISMATIC_JOINT ||
           path_.joint(j)->jointType() == cnoid::Link::JointType::SLIDE_JOINT){
          jacobian_.coeffRef(0,col)=omega[0];
          jacobian_.coeffRef(1,col)=omega[1];
          jacobian_.coeffRef(2,col)=omega[2];
          jacobian_.coeffRef(3,col)=0;
          jacobian_.coeffRef(4,col)=0;
          jacobian_.coeffRef(5,col)=0;
        }
        if(path_.joint(j)->jointType() == cnoid::Link::JointType::ROTATIONAL_JOINT ||
           path_.joint(j)->jointType() == cnoid::Link::JointType::REVOLUTE_JOINT){
          cnoid::Vector3 dp = omega.cross(target_p - path_.joint(j)->p());
          jacobian_.coeffRef(0,col)=dp[0];
          jacobian_.coeffRef(1,col)=dp[1];
          jacobian_.coeffRef(2,col)=dp[2];
          jacobian_.coeffRef(3,col)=omega[0];
          jacobian_.coeffRef(4,col)=omega[1];
          jacobian_.coeffRef(5,col)=omega[2];
        }
      }

      return jacobian_;
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPoint::calcRinv(cnoid::Link* parent, cnoid::Position& T_local) {
      Rinv_.resize(6,6);
      cnoid::Matrix3 Rtrans = ( parent->R() * T_local.linear() ).transpose();
      for(size_t j=0;j<3;j++){
        for(size_t k=0;k<3;k++){
          Rinv_.coeffRef(j,k) = Rtrans(j,k);
          Rinv_.coeffRef(3+j,3+k) = Rtrans(j,k);
        }
      }

      return Rinv_;
    }

  };
};
