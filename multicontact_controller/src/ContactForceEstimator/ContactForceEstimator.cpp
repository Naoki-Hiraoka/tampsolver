#include <multicontact_controller/ContactForceEstimator/ContactForceEstimator.h>
#include <cnoid/ForceSensor>
#include <cnoid/src/Body/InverseDynamics.h>

namespace multicontact_controller {

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
      std::vector<cnoid::Link*> child_links;
      for(cnoid::Link* link=forceSensors[i]->link()->child();link;link=link->sibling()){
        child_links.push_back(link);
      }
      for(size_t i=0;i<child_links.size();i++){
        parent_link->removeChild(child_links[i]);//childのsiblingが代わりにparentのchildになることに注意
      }

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

      for(size_t i=0;i<child_links.size();i++){
        linkYaw->appendChild(child_links[i]);
        child_links[i]->setOffsetTranslation(linkX->Tb().inverse() * child_links[i]->b());
      }

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

    forceSensorOffsets_.clear();
    for(size_t i=0;i<forceSensors.size();i++){
      forceSensorOffsets_.push_back(cnoid::Vector6::Zero());
    }

    rootForceOffset_ = cnoid::Vector6::Zero();

    return true;
  }

  bool ContactForceEstimator::setCandidatePoint(std::shared_ptr<ContactPointCFE> candidatepoint) {
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

    std::vector<std::shared_ptr<ContactPointCFE> >::iterator result = std::find_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPointCFE> x) { return x->name() == candidatepoint->name(); });
    if (result != candidatePoints_.end()){
      *result = candidatepoint;
    } else {
      candidatePoints_.push_back(candidatepoint);
    }

    changed_ = true;

    return true;
  }

  std::shared_ptr<ContactPointCFE> ContactForceEstimator::getCandidatePoint(const std::string& name) {
    std::vector<std::shared_ptr<ContactPointCFE> >::iterator result = std::find_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPointCFE> x) { return x->name() == name; });

    if (result == candidatePoints_.end()){
      return nullptr;
    } else {
      return *result;
    }
  }


  bool ContactForceEstimator::deleteCandidatePoint(const std::string& name) {
    std::vector<std::shared_ptr<ContactPointCFE> >::iterator result = std::remove_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPointCFE> x) { return x->name() == name; });
    candidatePoints_.erase(result, candidatePoints_.end());

    changed_ = true;
    return true;
  }

  bool ContactForceEstimator::clearCandidatePoints() {
    candidatePoints_.clear();
    return true;
  }

  bool ContactForceEstimator::estimateForce() {

    updateRobotState();

    /*
      g = sensors + J^F
     */
    cnoid::VectorX sensors = cnoid::VectorX::Zero(6+robot_->numJoints());
    for(size_t i=0;i<robot_->numJoints();i++){
      sensors(6+i) = robot_->joint(i)->u();
    }

    cnoid::VectorX g = cnoid::VectorX::Zero(6+robot_->numJoints());
    cnoid::Vector3 gravity(0, 0, 9.80665);
    robot_->rootLink()->dv() = gravity;
    robot_->rootLink()->dw().setZero();
    cnoid::Vector6 f = cnoid::calcInverseDynamics(robot_->rootLink()); //原点周り
    f.tail<3>() -= robot_->rootLink()->p().cross(f.head<3>()); // world系, rootlink周り
    g.head<6>() = f;
    for(size_t i=0;i<robot_->numJoints();i++){
      g(6+i) = robot_->joint(i)->u();
    }

    jacobian_.resize(6*candidatePoints_.size(),6+robot_->numJoints()); // contactpoint系、contactpoint周り
    for(size_t i=0;i<candidatePoints_.size();i++){
      jacobian_.middleRows(6*i,6) = candidatePoints_[i]->calcRinv() * candidatePoints_[i]->calcJacobian();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> jt = jacobian_.transpose();

    if(changed_){
      changed_ = false;
    }

    int task_idx=0;
    int dim = 6*candidatePoints_.size();
    const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());

    {
      // 力センサの値
      if(tasks_.size() <= task_idx) {
        std::shared_ptr<prioritized_qp::Task> task = std::make_shared<prioritized_qp::Task>();
        task->name() = "Force Sensor Task";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        //task->solver().settings()->setRho(1e-6);
        //task->solver().settings()->setAlpha(0.1);
        task->solver().settings()->setMaxIteration(4000);
        //task->solver().settings()->setRho(1e-6);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        //settings->eps_abs = 1e-05;
        //settings->eps_rel = 1e-05;
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;

        tasks_.push_back(task);
      }
      std::shared_ptr<prioritized_qp::Task> task = tasks_[task_idx];

      task->A().resize(6*forceSensors.size(),dim);
      task->b().resize(6*forceSensors.size());
      task->wa().resize(6*forceSensors.size());
      task->C().resize(0,dim);
      task->dl().resize(0);
      task->du().resize(0);
      task->wc().resize(0);
      task->w().resize(dim);

      for(size_t i=0;i<forceSensors.size();i++){
        const std::string& name = forceSensors[i]->name();
        int id = 6+robot_->link(name+"X")->jointId();
        task->A().middleRows(i*6,6) = jt.middleRows(id,6);
        task->b().segment<6>(i*6) = g.segment<6>(id) - sensors.segment<6>(id);
        for(size_t j=0;j<3;j++) task->wa()(i*6+j) = 1;
        for(size_t j=0;j<3;j++) task->wa()(i*6+3+j) = 1;
      }

      for(size_t i=0;i<candidatePoints_.size();i++){
        for(size_t j=0;j<3;j++) task->w()(i*6+j) = 1e-10;
        for(size_t j=0;j<3;j++) task->w()(i*6+3+j) = 1e-10;
      }

      task_idx++;
    }

    {
      // rootのつりあい
      if(tasks_.size() <= task_idx) {
        std::shared_ptr<prioritized_qp::Task> task = std::make_shared<prioritized_qp::Task>();
        task->name() = "Root Force Task";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        //task->solver().settings()->setRho(1e-6);
        //task->solver().settings()->setAlpha(0.1);
        task->solver().settings()->setMaxIteration(4000);
        //task->solver().settings()->setRho(1e-6);//いらない?
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        //settings->eps_abs = 1e-05;
        //settings->eps_rel = 1e-05;
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check

        task->toSolve() = true;

        tasks_.push_back(task);
      }
      std::shared_ptr<prioritized_qp::Task> task = tasks_[task_idx];

      task->A().resize(6,dim);
      task->b().resize(6);
      task->wa().resize(6);
      task->C().resize(0,dim);
      task->dl().resize(0);
      task->du().resize(0);
      task->wc().resize(0);
      task->w().resize(dim);

      task->A() = jt.topRows(6);
      task->b() = g.head<6>() - sensors.head<6>();
      for(size_t j=0;j<3;j++) task->wa()(j) = 1;
      for(size_t j=0;j<3;j++) task->wa()(3+j) = 1;

      for(size_t i=0;i<candidatePoints_.size();i++){
        for(size_t j=0;j<3;j++) task->w()(i*6+j) = 1e-10;
        for(size_t j=0;j<3;j++) task->w()(i*6+3+j) = 1e-10;
      }

      task_idx++;
    }

    {
      // 関節トルクの値
      if(tasks_.size() <= task_idx) {
        std::shared_ptr<prioritized_qp::Task> task = std::make_shared<prioritized_qp::Task>();
        task->name() = "Joint Torque Task";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        //task->solver().settings()->setRho(1e-6);
        //task->solver().settings()->setAlpha(0.1);
        task->solver().settings()->setMaxIteration(4000);
        //task->solver().settings()->setRho(1e-6);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        //settings->eps_abs = 1e-05;
        //settings->eps_rel = 1e-05;
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check

        task->toSolve() = true;

        tasks_.push_back(task);
      }
      std::shared_ptr<prioritized_qp::Task> task = tasks_[task_idx];

      task->A().resize(robot_org_->numJoints(),dim);
      task->b().resize(robot_org_->numJoints());
      task->wa().resize(robot_org_->numJoints());
      task->C().resize(0,dim);
      task->dl().resize(0);
      task->du().resize(0);
      task->wc().resize(0);
      task->w().resize(dim);

      for(size_t i=0;i<robot_org_->numJoints();i++){
        int id = 6+i;
        task->A().innerVector(i) = jt.innerVector(id);
        task->b()(i) = g(id) - sensors(id);
        task->wa()(i) = 1;
      }

      for(size_t i=0;i<candidatePoints_.size();i++){
        for(size_t j=0;j<3;j++) task->w()(i*6+j) = 1e-10;
        for(size_t j=0;j<3;j++) task->w()(i*6+3+j) = 1e-10;
      }

      task_idx++;
    }

    cnoid::VectorX result;

    if(dim!=0){
      bool solved = prioritized_qp::solve(tasks_,result);

      if(!solved) return false;
    }else{
      result = cnoid::VectorX(0);
    }

    for(size_t i=0;i<candidatePoints_.size();i++){
      candidatePoints_[i]->F() = result.segment<6>(i*6);
    }

    cnoid::VectorX forceOffsets = tasks_[0]->A() * result - tasks_[0]->b();
    for(size_t i=0;i<forceSensorOffsets_.size();i++){
      forceSensorOffsets_[i] = - forceOffsets.segment<6>(i*6); //符号が逆であることに注意
    }

    rootForceOffset_ = tasks_[1]->A() * result - tasks_[1]->b();

    return true;
  }

  cnoid::Vector6 ContactForceEstimator::getEstimatedForce(const std::string& name) {
    std::vector<std::shared_ptr<ContactPointCFE> >::iterator result = std::find_if(candidatePoints_.begin(), candidatePoints_.end(), [&](std::shared_ptr<ContactPointCFE> x) { return x->name() == name; });

    if (result == candidatePoints_.end()){
      return cnoid::Vector6::Zero();
    } else {
      return (*result)->F();
    }
  }

  cnoid::Vector6 ContactForceEstimator::getOffsetForce(const std::string& name) {
    const cnoid::DeviceList<cnoid::ForceSensor> forceSensors(robot_->devices());

    for(size_t i=0;i<forceSensors.size();i++){
      if(forceSensors[i]->name() == name) return forceSensorOffsets_[i];
    }
    return cnoid::Vector6::Zero();
  }

  bool ContactForceEstimator::updateRobotState() {

    // update parameters. 今のrobot_orgの状態をrobotに反映
    robot_->rootLink()->setMass(robot_org_->rootLink()->m());

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
       //符号が逆であることに注意
      robot_->link(name+"X")->u() = - F(0);
      robot_->link(name+"Y")->u() = - F(1);
      robot_->link(name+"Z")->u() = - F(2);
      robot_->link(name+"Roll")->u() = - F(3);
      robot_->link(name+"Pitch")->u() = - F(4);
      robot_->link(name+"Yaw")->u() = - F(5);
    }

    robot_->calcForwardKinematics(true,true);

    return true;
  }

};
