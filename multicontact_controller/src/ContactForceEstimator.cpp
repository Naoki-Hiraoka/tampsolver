#include <multicontact_controller/ContactForceEstimator.h>
#include <cnoid/ForceSensor>
#include <cnoid/src/Body/InverseDynamics.h>
#include <prioritized_qp/PrioritizedQPSolver.h>

namespace multicontact_controller {
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPoint::calcJacobian(){//world系,contactpoint周り
    // multi thread対応
    cnoid::Link* parent_est = parent_est_;
    cnoid::Position T_local_est = T_local_est_;

    if(path_.endLink() != parent_est){
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
      path_.setPath(parent_est);
      for(size_t j=0;j<path_.numJoints();j++){
        int col = 6+path_.joint(j)->jointId();
        tripletList.push_back(Eigen::Triplet<double>(0,col,1));
        tripletList.push_back(Eigen::Triplet<double>(1,col,1));
        tripletList.push_back(Eigen::Triplet<double>(2,col,1));
        tripletList.push_back(Eigen::Triplet<double>(3,col,1));
        tripletList.push_back(Eigen::Triplet<double>(4,col,1));
        tripletList.push_back(Eigen::Triplet<double>(5,col,1));
      }

      jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6+parent_est->body()->numJoints());
      jacobian_.setFromTriplets(tripletList.begin(), tripletList.end());
    }

    const cnoid::Position target_position = parent_est->T() * T_local_est;
    const cnoid::Vector3 target_p = target_position.translation();
    //root 6dof
    for(size_t j=0;j<6;j++){
      this->jacobian_.coeffRef(j,j) = 1;
    }
    cnoid::Vector3 dp = target_p - parent_est->body()->rootLink()->p();
    //  0     p[2] -p[1]
    // -p[2]  0     p[0]
    //  p[1] -p[0]  0
    this->jacobian_.coeffRef(0,4)=dp[2];
    this->jacobian_.coeffRef(0,5)=-dp[1];
    this->jacobian_.coeffRef(1,3)=-dp[2];
    this->jacobian_.coeffRef(1,5)=dp[0];
    this->jacobian_.coeffRef(2,3)=dp[1];
    this->jacobian_.coeffRef(2,4)=-dp[0];

    //joints
    for(size_t j=0;j<path_.numJoints();j++){
      int col = 6+path_.joint(j)->jointId();
      cnoid::Vector3 omega = path_.joint(j)->R() * path_.joint(j)->a();
      if(!path_.isJointDownward(j)) omega = -omega;
      if(path_.joint(j)->jointType() == cnoid::Link::JointType::PRISMATIC_JOINT ||
         path_.joint(j)->jointType() == cnoid::Link::JointType::SLIDE_JOINT){
        this->jacobian_.coeffRef(0,col)=omega[0];
        this->jacobian_.coeffRef(1,col)=omega[1];
        this->jacobian_.coeffRef(2,col)=omega[2];
        this->jacobian_.coeffRef(3,col)=0;
        this->jacobian_.coeffRef(4,col)=0;
        this->jacobian_.coeffRef(5,col)=0;
      }
      if(path_.joint(j)->jointType() == cnoid::Link::JointType::ROTATIONAL_JOINT ||
         path_.joint(j)->jointType() == cnoid::Link::JointType::REVOLUTE_JOINT){
        cnoid::Vector3 dp = omega.cross(target_p - path_.joint(j)->p());
        this->jacobian_.coeffRef(0,col)=dp[0];
        this->jacobian_.coeffRef(1,col)=dp[1];
        this->jacobian_.coeffRef(2,col)=dp[2];
        this->jacobian_.coeffRef(3,col)=omega[0];
        this->jacobian_.coeffRef(4,col)=omega[1];
        this->jacobian_.coeffRef(5,col)=omega[2];
      }
    }
  }

  bool ContactForceEstimator::setRobot(const cnoid::Body* robot) {
    robot_org_ = robot;

    robot_ = std::shared_ptr<cnoid::Body>(robot_org_->clone());
    int nj = robot_->numJoints();
    const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
    for(size_t i=0; i<forceSensors.size(); i++){
      /*
        <BEFORE>
        parent_link - mass - geometry - forcesensor
        | Tb
        child_link

        <AFTER>
        parent_link - geometry - forcesensor
        | local_p
        X
        |
        Y
        |
        Z
        |
        Roll
        |
        Pitch
        |
        Yaw - mass
        | local_p^-1 * Tb
        child_link

        注: TbのR成分はcalcForwardKinematicsで考慮されないので、代わりに回転軸を傾ける
       */
      const std::string& name = forceSensors[i]->name();
      cnoid::Link* parent_link = forceSensors[i]->link();
      cnoid::Link* child_link = forceSensors[i]->link()->child();
      parent_link->removeChild(child_link);

      cnoid::Link* linkX = new cnoid::Link();
      linkX->setName(name+"X");
      parent_link->appendChild(linkX);
      linkX->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
      linkX->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitX());
      linkX->setJointId(nj++);//jointIdがないとcalcMassMatrixで考慮されない
      linkX->setOffsetTranslation(forceSensors[i]->p_local());
      cnoid::Link* linkY = new cnoid::Link();
      linkY->setName(name+"Y");
      linkX->appendChild(linkY);
      linkY->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
      linkY->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitY());
      linkY->setJointId(nj++);
      cnoid::Link* linkZ = new cnoid::Link();
      linkZ->setName(name+"Z");
      linkY->appendChild(linkZ);
      linkZ->setJointType(cnoid::Link::JointType::PRISMATIC_JOINT);
      linkZ->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitZ());
      linkZ->setJointId(nj++);
      cnoid::Link* linkRoll = new cnoid::Link();
      linkRoll->setName(name+"Roll");
      linkZ->appendChild(linkRoll);
      linkRoll->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
      linkRoll->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitX());
      linkRoll->setJointId(nj++);
      cnoid::Link* linkPitch = new cnoid::Link();
      linkPitch->setName(name+"Pitch");
      linkRoll->appendChild(linkPitch);
      linkPitch->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
      linkPitch->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitY());
      linkPitch->setJointId(nj++);
      cnoid::Link* linkYaw = new cnoid::Link();
      linkYaw->setName(name+"Yaw");
      linkPitch->appendChild(linkYaw);
      linkYaw->setJointType(cnoid::Link::JointType::ROTATIONAL_JOINT);
      linkYaw->setJointAxis(forceSensors[i]->R_local() * cnoid::Vector3::UnitZ());
      linkYaw->setJointId(nj++);

      linkYaw->appendChild(child_link);
      child_link->T() = linkX->Tb().inverse() * child_link->T();

      linkYaw->setCenterOfMass(parent_link->centerOfMass() - forceSensors[i]->p_local());
      linkYaw->setMass(parent_link->mass());
      linkYaw->setInertia(parent_link->I());
      parent_link->setCenterOfMass(cnoid::Vector3::Zero());
      parent_link->setMass(0);
      parent_link->setInertia(cnoid::Matrix3::Zero());
    }

    robot_->setRootLink(robot_->rootLink());

    for(size_t i=0;i<candidatePoints_.size();i++){
      if(!setCandidatePoint(candidatePoints_[i])){
        deleteCandidatePoint(candidatePoints_[i]->name());
      }
    }

    return true;
  }

  bool ContactForceEstimator::setCandidatePoint(std::shared_ptr<ContactPoint> candidatepoint) {
    cnoid::Link* parent_link = robot_->link(candidatepoint->parent()->name());
    cnoid::Position T_local = candidatepoint->T_local();
    if(!parent_link) return false;
    const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());
    for(size_t i=0;i<forceSensors.size();i++){
      if(forceSensors[i]->link()->name() == parent_link->name()){
        parent_link = robot_->link(forceSensors[i]->name()+"Yaw");
        cnoid::Position T_sensor; T_sensor.setIdentity();
        T_sensor.translation() = forceSensors[i]->p_local();
        T_local = T_sensor.inverse() *  T_local;
        break;
      }
    }
    candidatepoint->parent_est() = parent_link;
    candidatepoint->T_local_est() = T_local;

    std::vector<std::shared_ptr<ContactPoint> >::iterator result = std::find_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPoint> x) { return x->name() == candidatepoint->name(); });
    if (result != candidatePoints_.end()){
      *result = candidatepoint;
    } else {
      candidatePoints_.push_back(candidatepoint);
    }

    changed_ = true;

    return true;
  }

  std::shared_ptr<ContactPoint> ContactForceEstimator::getCandidatePoint(const std::string& name) {
    std::vector<std::shared_ptr<ContactPoint> >::iterator result = std::find_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPoint> x) { return x->name() == name; });

    if (result != candidatePoints_.end()){
      return nullptr;
    } else {
      return *result;
    }
  }


  bool ContactForceEstimator::deleteCandidatePoint(const std::string& name) {
    std::vector<std::shared_ptr<ContactPoint> >::iterator result = std::remove_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPoint> x) { return x->name() == name; });
    candidatePoints_.erase(result, candidatePoints_.end());

    changed_ = true;
    return true;
  }

  bool ContactForceEstimator::estimateForce() {

    // update parameters
    robot_->rootLink()->T() = robot_org_->rootLink()->T();
    for(size_t i=0;i<robot_org_->numJoints();i++){
      const cnoid::Link* joint_org = robot_org_->joint(i);
      cnoid::Link* joint = robot_->link(joint_org->name());
      joint->q() = joint_org->q();
      joint->dq() = joint_org->dq();
      joint->ddq() = joint_org->ddq();
      joint->u() = joint_org->u();
    }

    const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_org_->devices());
    for(size_t i=0;i<forceSensors.size();i++){
      const std::string& name = forceSensors[i]->name();
      const cnoid::Vector6& F = forceSensors[i]->F();
      robot_->link(name+"X")->u() = F(0);
      robot_->link(name+"Y")->u() = F(1);
      robot_->link(name+"Z")->u() = F(2);
      robot_->link(name+"Roll")->u() = F(3);
      robot_->link(name+"Pitch")->u() = F(4);
      robot_->link(name+"Yaw")->u() = F(5);
    }

    robot_->calcForwardKinematics(true,true);

    /*
      g = sensors + J^F
     */
    cnoid::VectorX sensors = cnoid::VectorX::Zero(6+robot_->numJoints());
    for(size_t i=0;i<robot_->numJoints();i++){
      sensors(6+i) = robot_->joint(i)->u();
    }

    cnoid::VectorX g = cnoid::VectorX::Zero(6+robot_->numJoints());
    cnoid::Vector6 f = cnoid::calcInverseDynamics(robot_->rootLink()); //原点周り
    f.tail<3>() -= robot_->rootLink()->p().cross(f.head<3>()); // world系, rootlink周り
    g.head<6>() = f;
    for(size_t i=0;i<robot_->numJoints();i++){
      g(6+i) = robot_->joint(i)->u();
    }

    jacobian_.resize(6*candidatePoints_.size(),6+robot_->numJoints()); // contactpoint系、contactpoint周り
    Eigen::SparseMatrix<double,Eigen::RowMajor> Rinv(6,6);
    for(size_t i=0;i<candidatePoints_.size();i++){
      cnoid::Matrix3 Rtrans = (candidatePoints_[i]->parent_est()->R() * candidatePoints_[i]->T_local_est().linear() ).transpose();
      for(size_t j=0;j<3;j++){
        for(size_t k=0;k<3;k++){
          Rinv.coeffRef(j,k) = Rtrans(j,k);
          Rinv.coeffRef(3+j,3+k) = Rtrans(j,k);
        }
      }
      jacobian_.middleRows(6*i,6) = Rinv * candidatePoints_[i]->calcJacobian();
    }

    if(changed_){
      changed_ = false;
    }

    return true;
  }

  cnoid::Vector6 ContactForceEstimator::getEstimatedForce(const std::string& name) {
    return cnoid::Vector6::Zero();
  }

  bool ContactForceEstimator::removeForceSensorOffset(double time) {
    return true;
  }
  bool ContactForceEstimator::remoteRootForceOffset(double time) {
    return true;
  }
  bool ContactForceEstimator::remoteJointTorqueOffset(double time) {
    return true;
  }

};
