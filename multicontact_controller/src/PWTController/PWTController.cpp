#include <multicontact_controller/PWTController/PWTController.h>

#include <cnoid/EigenUtil>


namespace multicontact_controller {
  // S J dqa = 0 となるSを返す. ? x 6
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPointPWTC::selectMatrixForKinematicsConstraint(){
    if(this->state() == "CONTACT" || this->state() == "TOWARD_BREAK_CONTACT"){
      return this->contact()->selectMatrix();
    }else{
      selectMatrixForKinematicsConstraint_.resize(0,6);
      return selectMatrixForKinematicsConstraint_;
    }
  }

  // KinematicsConstraint による拘束力の接触維持に必要な制約を返す.
  // 各行は無次元化されている
  void ContactPointPWTC::contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    if(this->state() == "CONTACT"){
      this->contact()->getContactConstraint(A,b,wa,C,dl,du,wc);
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
    }else if(this->state() == "TOWARD_BREAK_CONTACT"){
      this->contact()->getContactConstraint(A,b,wa,C,dl,du,wc,true);//allow_break_contact
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
    }else{
      A.resize(0,0);
      b.resize(0);
      wa.resize(0);
      C.resize(0,0);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }

    return;
  }

  // KinematicsConstraint による拘束力の目標値を返す。主に接触解除時用
  void ContactPointPWTC::desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(this->state() == "TOWARD_BREAK_CONTACT"){
      this->contact()->getBreakContactConstraint(A,b,wa,C,dl,du,wc);
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
    }else{
      A.resize(0,0);
      b.resize(0);
      wa.resize(0);
      C.resize(0,0);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }

    return;
  }

  // KinematicsConstraint による拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
  //各行は無次元化される
  void ContactPointPWTC::bestEffortForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(this->state() == "CONTACT" || this->state() == "TOWARD_BREAK_CONTACT"){
      this->contact()->getStabilityConstraint(A,b,wa,C,dl,du,wc);
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
    }else{
      A.resize(0,0);
      b.resize(0);
      wa.resize(0);
      C.resize(0,0);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }

    return;
  }

  // 位置の目標値を返す。主に遊脚用. colは[root6dof + numJoint]
  // 各行はm, rad
  void ContactPointPWTC::desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(this->state() == "AIR" || this->state() == "NEAR_CONTACT"){
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local;
      this->interaction()->desiredPositionConstraint(A_local,b,wa,C_local,dl,du,wc);
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& J = this->calcJacobian();//world系,contactpoint周り
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Rinv = this->calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
      A = A_local * Rinv * J;
      C = C_local * Rinv * J;
    } else if(this->state() == "TOWARD_MAKE_CONTACT"){
      cnoid::Vector6 contactDirection = this->contact()->selectMatrix().transpose() * this->contact()->contactDirection();
      if(contactDirection.head<3>().norm() > 0){
        contactDirection.head<3>() = contactDirection.head<3>().normalized() * this->interaction()->v_limit() * this->interaction()->dt();
        this->interaction()->T_ref().translation() += contactDirection.head<3>();
      }
      if(contactDirection.tail<3>().norm() > 0){
        contactDirection.tail<3>() = contactDirection.tail<3>().normalized() * this->interaction()->w_limit() * this->interaction()->dt();
        this->interaction()->T_ref().linear() = (this->interaction()->T_ref().linear() * cnoid::AngleAxis(contactDirection.tail<3>().norm(),contactDirection.tail<3>().normalized())).eval();
      }
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local;
      this->interaction()->desiredPositionConstraint(A_local,b,wa,C_local,dl,du,wc);
      Eigen::SparseMatrix<double,Eigen::RowMajor> J = this->calcJacobian();
      A = A_local * J;
      C = C_local * J;
    }else{
      A.resize(0,6+this->parent()->body()->numJoints());
      b.resize(0);
      wa.resize(0);
      C.resize(0,6+this->parent()->body()->numJoints());
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }

    return;
  }

  std::vector<cnoid::SgNodePtr> ContactPointPWTC::getDrawOnObjects(){
    std::vector<cnoid::SgNodePtr> drawOnObjects;
    if(state_ == "CONTACT" || state_ == "TOWARD_BREAK_CONTACT"){
      if(contact_ && parent_){
        std::vector<cnoid::SgNodePtr> objects = contact_->getDrawOnObjects(parent_->T() * T_local_);
        std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
      }
    }
    if(state_ == "TOWARD_MAKE_CONTACT" || state_ == "NEAR_CONTACT" || state_ == "AIR"){
      if(interaction_ && parent_){
        std::vector<cnoid::SgNodePtr> objects = interaction_->getDrawOnObjects();
        std::copy(objects.begin(), objects.end(), std::back_inserter(drawOnObjects));
      }
    }
    return drawOnObjects;
  }

  JointInfo::JointInfo()
    : name_(),
      joint_(nullptr),
      controllable_(false),
      care_torque_(false),
      command_angle_(0.0),
      dt_(0.02),
      coil_temperature_limit_(150.0),
      housing_temperature_(25.0),
      coil_temperature_(25.0),
      maximum_effort_soft_(std::numeric_limits<double>::max()),
      maximum_effort_hard_(std::numeric_limits<double>::max()),
      balance_effort_(std::numeric_limits<double>::max()),
      remaining_time_(std::numeric_limits<double>::max()),
      pgain_(1e4),
      hardware_pgain_(1e0),
      ulimit_(std::numeric_limits<double>::max()),
      llimit_(-std::numeric_limits<double>::max()),
      uvlimit_(std::numeric_limits<double>::max()),
      lvlimit_(-std::numeric_limits<double>::max())
  {
  }

  // 指令関節角度上下限に関する制約を返す.破損防止
  void JointInfo::JointAngleConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    double ulimit = joint_->q_upper();
    double llimit = joint_->q_lower();

    // angle
    if(jointLimitTable_){
      double min_angle = jointLimitTable_->getLlimit(jointLimitTableTargetJointInfo_.lock()->command_angle()) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
      double max_angle = jointLimitTable_->getUlimit(jointLimitTableTargetJointInfo_.lock()->command_angle()) + 0.001;//今回のqpの結果超えることを防ぐため、少しマージン
      ulimit = std::max(llimit, std::min(ulimit, max_angle - command_angle_)); //上限が下限を下回ることを防ぐ
      llimit = std::min(ulimit, std::max(llimit, min_angle - command_angle_));
    }else{
      double min_angle = joint_->q_lower() + 0.0001;//少しマージン
      double max_angle = joint_->q_upper() - 0.0001;//少しマージン
      ulimit = std::max(llimit, std::min(ulimit, max_angle - command_angle_)); //上限が下限を下回ることを防ぐ
      llimit = std::min(ulimit, std::max(llimit, min_angle - command_angle_));
    }

    A.resize(0,1);
    b.resize(0);
    wa.resize(0);
    C.resize(1,1);C.coeffRef(0,0) = 1.0;
    dl.resize(1);dl[0] = llimit;
    du.resize(1);du[0] = ulimit;
    wc.resize(1);wc[0] = 1.0;
    return;
  }

  // 指令関節角速度上下限に関する制約を返す.破損防止
  void JointInfo::JointVelocityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    // velocity
    double ulimit = joint_->q_upper() * dt_;
    double llimit = joint_->q_lower() * dt_;
    ulimit = std::max(llimit, std::min(ulimit, joint_->dq_upper() * dt_ - 0.000175));// 0.01 deg / sec (same as SoftErrorLimiter)
    llimit = std::min(ulimit, std::max(llimit, joint_->dq_lower() * dt_ + 0.000175));// 0.01 deg / sec (same as SoftErrorLimiter)
    ulimit = std::max(llimit, std::min(ulimit, uvlimit_)); //上限が下限を下回ることを防ぐ
    llimit = std::min(ulimit, std::max(llimit, lvlimit_));

    A.resize(0,1);
    b.resize(0);
    wa.resize(0);
    C.resize(1,1);C.coeffRef(0,0) = 1.0;
    dl.resize(1);dl[0] = llimit;
    du.resize(1);du[0] = ulimit;
    wc.resize(1);wc[0] = 1.0;
    return;
  }

  // M \tau によってこのJointの成分を抽出できるM(select matrix). \tauは[numJoints]
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& JointInfo::torqueSelectMatrix(){
    if(torqueSelectMatrix_.cols() != joint_->body()->numJoints() || torqueSelectMatrix_.rows() != 1){
      torqueSelectMatrix_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(1,joint_->body()->numJoints());
      torqueSelectMatrix_.insert(0,joint_->jointId()) = 1.0;
    }
    return torqueSelectMatrix_;
  }

  // 関節トルク上下限に関する制約を返す.破損防止
  // 各行は無次元化されている
  void JointInfo::JointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    if(care_torque_){
      double max_torque = joint_->info<double>("climit") * joint_->info<double>("torqueConst") * joint_->info<double>("gearRatio");
      double scale = std::max(std::min(maximum_effort_hard_, max_torque), 1e-4);
      A.resize(0,1);
      b.resize(0);
      wa.resize(0);
      C.resize(1,1); C.coeffRef(0,0) = 1.0/scale;
      dl.resize(1);
      du.resize(1);
      wc.resize(1);

      du[0] = std::min(maximum_effort_hard_, max_torque)/scale - C.coeffRef(0,0) * joint_->u();
      dl[0] = std::max(-maximum_effort_hard_, -max_torque)/scale - C.coeffRef(0,0) * joint_->u();
      wc[0] = 1.0;
    }else{
      A.resize(0,1);
      b.resize(0);
      wa.resize(0);
      C.resize(0,1);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }
    return;
  }

  // 各行は無次元化されている
  void JointInfo::bestEffortJointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(care_torque_){
      double max_torque = joint_->info<double>("climit") * joint_->info<double>("torqueConst") * joint_->info<double>("gearRatio");
      double scale = std::max(std::min(maximum_effort_soft_, max_torque), 1e-4);
      A.resize(1,1); A.coeffRef(0,0) = 1.0/scale;
      b.resize(1);
      wa.resize(1);
      C.resize(0,1);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);

      b[0] = 0 - A.coeffRef(0,0) * joint_->u();
      wa[0] = 1.0;
    }else{
      A.resize(0,1);
      b.resize(0);
      wa.resize(0);
      C.resize(0,1);
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }

    return;
  }

  bool PWTController::calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                     std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                     std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& pclCollisions,
                                     double dt){

    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > Js;Js.reserve(contactPoints.size());
    for(size_t i=0;i<contactPoints.size();i++) Js.emplace_back(contactPoints[i]->calcJacobian());

    // calc PWT Jacobian
    Eigen::SparseMatrix<double,Eigen::RowMajor> Dqa;
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Dwas;
    Eigen::SparseMatrix<double,Eigen::RowMajor> Dtaua;
    if(!this->calcPWTJacobian(Dqa,
                              Dwas,
                              Dtaua,
                              robot_,
                              jointInfos_,
                              torqueJacobianCalculator_,
                              contactPoints,
                              Js,
                              sv_ratio_)){
      std::cerr << "calcPWTJacobian failed" << std::endl;
      return false;
    }
    // std::cerr << "Dqa" << std::endl;
    // std::cerr << Dqa << std::endl;
    // std::cerr << "Dwas" << std::endl;
    // for(size_t i=0;i<Dwas.size();i++){
    //   std::cerr << Dwas[i] << std::endl;
    // }
    // std::cerr << "Dtaua" << std::endl;
    // std::cerr << Dtaua << std::endl;

    // setup tasks
    std::vector<std::shared_ptr<prioritized_qp::Task> > tasks;

    {
      // priority 0
      std::shared_ptr<prioritized_qp::Task> task0;
      if(!this->setupTask0(task0, robot_, jointInfos_, k0_, dt)){
        std::cerr << "setupTask0 failed" << std::endl;
        return false;
      }
      tasks.push_back(task0);
    }

    {
      // priority 0.1
      std::shared_ptr<prioritized_qp::Task> task0_1;
      if(!this->setupTask0_1(task0_1,
                             robot_,
                             jointInfos_,
                             selfCollisions,
                             Dqa,
                             tolerance0_1_,
                             k0_1_,
                             dt,
                             w0_1_,
                             we0_1_)){
        std::cerr << "setupTask0.1 failed" << std::endl;
        return false;
      }
      tasks.push_back(task0_1);
    }

    {
      // priority 1
      std::shared_ptr<prioritized_qp::Task> task1;
      if(!this->setupTask1(task1,
                           robot_,
                           jointInfos_,
                           contactPoints,
                           Dwas,
                           Dtaua,
                           w_scale1_,
                           tau_scale1_,
                           k1_,
                           dt,
                           w1_,
                           we1_)){
        std::cerr << "setupTask1 failed" << std::endl;
        return false;
      }
      tasks.push_back(task1);
    }

    {
      // priority 1.1
      std::shared_ptr<prioritized_qp::Task> task1_1;
      if(!this->setupTask1_1(task1_1,
                             robot_,
                             jointInfos_,
                             pclCollisions,
                             Dqa,
                             tolerance1_1_,
                             k1_1_,
                             dt,
                             w1_1_,
                             we1_1_)){
        std::cerr << "setupTask1.1 failed" << std::endl;
        return false;
      }
      tasks.push_back(task1_1);
    }

    {
      // priority 2
      std::shared_ptr<prioritized_qp::Task> task2;
      if(!this->setupTask2(task2,
                           robot_,
                           jointInfos_,
                           contactPoints,
                           Dqa,
                           dt,
                           w2_,
                           we2_)){
        std::cerr << "setupTask2 failed" << std::endl;
        return false;
      }
      tasks.push_back(task2);
    }

    {
      // priority 2.5
      std::shared_ptr<prioritized_qp::Task> task2_5;
      if(!this->setupTask2_5(task2_5,
                             robot_,
                             jointInfos_,
                             contactPoints,
                             Dwas,
                             w_scale2_5_,
                             k2_5_,
                             dt,
                             w2_5_,
                             we2_5_)){
        std::cerr << "setupTask2_5 failed" << std::endl;
        return false;
      }
      tasks.push_back(task2_5);
    }

    {
      // priority 3
      std::shared_ptr<prioritized_qp::Task> task3Helper;
      std::shared_ptr<prioritized_qp::Task> task3;
      if(!this->setupTask3(task3Helper,
                           task3,
                           robot_,
                           jointInfos_,
                           contactPoints,
                           Dwas,
                           Dtaua,
                           w_scale3_,
                           tau_scale3_,
                           taumax_weight3_,
                           k3_,
                           dt,
                           w3_)){
        std::cerr << "setupTask3 failed" << std::endl;
        return false;
      }
      tasks.push_back(task3Helper);
      tasks.push_back(task3);
    }

    // Solve
    cnoid::VectorX result;
    bool solved = prioritized_qp::solve(tasks,result);
    if(!solved) {
      std::cerr << "prioritized_qp::solve failed" << std::endl;
      return false;
    }

    // reflect result
    {
      cnoid::VectorX dqa = Dqa * result.head(Dqa.cols());
      for(size_t i=0;i<contactPoints.size();i++) contactPoints[i]->update(dqa);
    }
    {
      size_t idx = 0;
      for(size_t i=0;i<jointInfos_.size();i++){
        if(jointInfos_[i]->controllable()){
          jointInfos_[i]->command_angle() += result[idx];
          idx++;
        }
      }

    }

    if(debug_print_){
      std::cerr << "result" << result << std::endl;
    }

    return true;
  }

  bool PWTController::calcPWTJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,//返り値
                                      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,//返り値
                                      Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,//返り値
                                      cnoid::Body* robot,
                                      std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                      cnoidbodyutils::TorqueJacobianCalculator& torqueJacobianCalculator,
                                      std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > >& Js,
                                      double sv_ratio){

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dg = torqueJacobianCalculator.calcDg();

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& DJw = torqueJacobianCalculator.calcDJw(contactPoints); // F = 0 のcontactpointをここで入れるのは計算コストの無駄かもしれない

    for(size_t i=0;i<robot->numJoints();i++){
      Ka_.coeffRef(6+i,6+i) = jointInfos[i]->pgain();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> Jbal(0,6+robot->numJoints());
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Jbals;
    for(size_t i=0;i<contactPoints.size();i++) {
      Jbals.push_back(contactPoints[i]->selectMatrixForKinematicsConstraint() * Js[i].get());
    }
    cnoidbodyutils::appendRow(Jbals, Jbal);

    Eigen::SparseMatrix<double,Eigen::RowMajor> X(6+robot->numJoints()+Jbal.rows(),6+robot->numJoints()+Jbal.rows());
    {
      Eigen::SparseMatrix<double,Eigen::RowMajor> Xleftupper = Dg - DJw + Ka_;
      Eigen::SparseMatrix<double,Eigen::ColMajor> Xupper(6+robot->numJoints(),6+robot->numJoints()+Jbal.rows());
      Xupper.leftCols(6+robot->numJoints()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(Xleftupper);
      Xupper.rightCols(Jbal.rows()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(-Jbal.transpose());
      Eigen::SparseMatrix<double,Eigen::ColMajor> Xlower(Jbal.rows(),6+robot->numJoints()+Jbal.rows());
      Xlower.leftCols(6+robot->numJoints()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(Jbal);
      X.topRows(Xupper.rows()) = Eigen::SparseMatrix<double,Eigen::RowMajor>(Xupper);
      X.bottomRows(Xlower.rows()) = Eigen::SparseMatrix<double,Eigen::RowMajor>(Xlower);
    }

    cnoid::MatrixXd Xinv;
    cnoidbodyutils::calcPseudoInverse(X,Xinv,sv_ratio);

    Eigen::SparseMatrix<double,Eigen::RowMajor> Yu(6+robot->numJoints(),Xinv.cols());
    Eigen::SparseMatrix<double,Eigen::RowMajor> Yl(Jbal.rows(),Xinv.cols());
    for(size_t i=0;i<Yu.rows();i++){
      for(size_t j=0;j<Yu.cols();j++){
        Yu.insert(i,j) = Xinv(i,j);
      }
    }
    for(size_t i=0;i<Yl.rows();i++){
      for(size_t j=0;j<Yl.cols();j++){
        Yl.insert(i,j) = Xinv(Yu.rows()+i,j);
      }
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> Kc;
    {
      size_t num_cols = 0;
      for(size_t i=0;i<robot->numJoints();i++){
        if(jointInfos[i]->controllable()) num_cols++;
      }
      Kc.resize(Xinv.cols(),num_cols);
      size_t idx = 0;
      for(size_t i=0;i<robot->numJoints();i++){
        if(jointInfos[i]->controllable()){
          Kc.insert(6+i,idx) = jointInfos[i]->pgain();
          idx++;
        }
      }
    }

    Dqa = Yu * Kc;
    {
      Eigen::SparseMatrix<double, Eigen::RowMajor> Dwa = Yl * Kc;
      Dwas.resize(contactPoints.size());
      size_t idx = 0;
      for(size_t i=0;i<contactPoints.size();i++){
        Dwas[i] = Dwa.middleRows(idx,Jbals[i].rows());
        idx += Jbals[i].rows();
      }
    }
    Dtaua = (Kc - Eigen::SparseMatrix<double,Eigen::RowMajor>(Ka_ * Dqa)).bottomRows(robot->numJoints());
    return true;
  }

  bool PWTController::setupTask0(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                 double k,
                                 double dt){
      if(!this->task0_) {
        this->task0_ = std::make_shared<prioritized_qp::Task>();
        task = this->task0_;
        task->name() = "Task0: Joint Limit, Self Collision";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = false; // 解が必ず存在すると仮定し解かない
      }else{
        task = this->task0_;
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      // 以下の制約を同時に満たす解が存在しないことは想定していない。
      // すなわち、関節角度上下限の外での動作や、自己干渉中の動作は現在不可能
      {
        // joint angle limit. command angleに対して
        size_t idx = 0;
        for(size_t i=0;i<jointInfos.size();i++){
          if(jointInfos[i]->controllable()) {
            Eigen::SparseMatrix<double, Eigen::RowMajor> S(1,cols); S.insert(0,idx) = k / dt; // velocity damper
            Eigen::SparseMatrix<double,Eigen::RowMajor> A;
            cnoid::VectorX b;
            cnoid::VectorX wa;
            Eigen::SparseMatrix<double,Eigen::RowMajor> C;
            cnoid::VectorX dl;
            cnoid::VectorX du;
            cnoid::VectorX wc;
            jointInfos[i]->JointAngleConstraint(A,b,wa,C,dl,du,wc);
            As.push_back(A*S);
            bs.push_back(b);
            was.push_back(wa);
            Cs.push_back(C*S);
            dls.push_back(dl);
            dus.push_back(du);
            wcs.push_back(wc);
            idx++;
          }
        }
      }

      {
        // joint velocity limit. command angleに対して
        size_t idx = 0;
        for(size_t i=0;i<jointInfos.size();i++){
          if(jointInfos[i]->controllable()) {
            Eigen::SparseMatrix<double, Eigen::RowMajor> S(1,cols); S.insert(0,idx) = 1.0;
            Eigen::SparseMatrix<double,Eigen::RowMajor> A;
            cnoid::VectorX b;
            cnoid::VectorX wa;
            Eigen::SparseMatrix<double,Eigen::RowMajor> C;
            cnoid::VectorX dl;
            cnoid::VectorX du;
            cnoid::VectorX wc;
            jointInfos[i]->JointVelocityConstraint(A,b,wa,C,dl,du,wc);
            As.push_back(A*S);
            bs.push_back(b);
            was.push_back(wa);
            Cs.push_back(C*S);
            dls.push_back(dl);
            dus.push_back(du);
            wcs.push_back(wc);
            idx++;
          }
        }
      }

      {
        // self collision TODO
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      task->w().resize(cols);//解かないので使わない
      for(size_t i=0;i<task->w().size();i++) task->w()[i] = 1.0;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl << task->A() << std::endl;
        std::cerr << "b" << std::endl << task->b() << std::endl;
        std::cerr << "C" << std::endl << task->C() << std::endl;
        std::cerr << "dl" << std::endl << task->dl() << std::endl;
        std::cerr << "du" << std::endl << task->du() << std::endl;
        std::cerr << "wa" << std::endl << task->wa() << std::endl;
        std::cerr << "wc" << std::endl << task->wc() << std::endl;
        std::cerr << "w" << std::endl << task->w() << std::endl;
      }

      return true;
  }

  bool PWTController::setupTask0_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                   std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                   const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                                   double tolerance,
                                   double k,
                                   double dt,
                                   double w,
                                   double we){
      if(!this->task0_1_) {
        this->task0_1_ = std::make_shared<prioritized_qp::Task>();
        task = this->task0_1_;
        task->name() = "Task0.1: Self Collision";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task0_1_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      {
        // self Collision
        // 各行mのオーダ
        for(size_t i=0;i<selfCollisions.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          selfCollisions[i]->tolerance() = tolerance;
          selfCollisions[i]->getCollisionConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dqa);
          bs.push_back(b);
          was.push_back(wa);
          Cs.push_back(C*Dqa);
          dls.push_back(dl);
          dus.push_back(du);
          wcs.push_back(wc);
        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;

      // damping factor
      double damping_factor = this->dampingFactor(w,
                                                  we,
                                                  task->b(),
                                                  task->wa(),
                                                  task->dl(),
                                                  task->du(),
                                                  task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
      }

      return true;
  }


  bool PWTController::setupTask1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                                 double w_scale,
                                 double tau_scale,
                                 double k,
                                 double dt,
                                 double w,
                                 double we){
      if(!this->task1_) {
        this->task1_ = std::make_shared<prioritized_qp::Task>();
        task = this->task1_;
        task->name() = "Task1: Contact Force, Joint Torque";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task1_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      {
        // contact force constraint
        // 各行は無次元化された上でw_scaleで割られてm,radのオーダにする
        for(size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          contactPoints[i]->contactForceConstraintForKinematicsConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dwas[i] / w_scale);
          bs.push_back(b / w_scale);
          was.push_back(wa);
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale);
          dus.push_back(du / w_scale);
          wcs.push_back(wc);
        }
      }

      {
        // joint torque limit
        // 各行は無次元化された上でtau_scaleで割られてm,radのオーダにする
        for(size_t i=0;i<jointInfos.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          jointInfos[i]->JointTorqueConstraint(A,b,wa,C,dl,du,wc);
          const Eigen::SparseMatrix<double, Eigen::RowMajor>& S = jointInfos[i]->torqueSelectMatrix();
          As.push_back(A*S*Dtaua / tau_scale);
          bs.push_back(b / tau_scale);
          was.push_back(wa);
          Cs.push_back(C*S*Dtaua / tau_scale);
          dls.push_back(dl / tau_scale);
          dus.push_back(du / tau_scale);
          wcs.push_back(wc);
        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;

      // damping factor
      double damping_factor = this->dampingFactor(w,
                                                  we,
                                                  task->b(),
                                                  task->wa(),
                                                  task->dl(),
                                                  task->du(),
                                                  task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
      }

      return true;
  }

  bool PWTController::setupTask1_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                   std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& pclCollisions,
                                   const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                                   double tolerance,
                                   double k,
                                   double dt,
                                   double w,
                                   double we){
      if(!this->task1_1_) {
        this->task1_1_ = std::make_shared<prioritized_qp::Task>();
        task = this->task1_1_;
        task->name() = "Task1.1: PCL Collision";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task1_1_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      {
        // PCL Collision
        // 各行mのオーダ
        for(size_t i=0;i<pclCollisions.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          pclCollisions[i]->tolerance() = tolerance;
          pclCollisions[i]->getCollisionConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dqa);
          bs.push_back(b);
          was.push_back(wa);
          Cs.push_back(C*Dqa);
          dls.push_back(dl);
          dus.push_back(du);
          wcs.push_back(wc);
        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;

      // damping factor
      double damping_factor = this->dampingFactor(w,
                                                  we,
                                                  task->b(),
                                                  task->wa(),
                                                  task->dl(),
                                                  task->du(),
                                                  task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
      }

      return true;
  }

  bool PWTController::setupTask2(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                                 double dt,
                                 double w,
                                 double we){
      if(!this->task2_) {
        this->task2_ = std::make_shared<prioritized_qp::Task>();
        task = this->task2_;
        task->name() = "Task2: Interacting EndEffector";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task2_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      {
        // interaction end_effector
        // 各行はm,radのオーダ
        for(size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          contactPoints[i]->desiredPositionConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dqa);
          bs.push_back(b);
          was.push_back(wa);
          Cs.push_back(C*Dqa);
          dls.push_back(dl);
          dus.push_back(du);
          wcs.push_back(wc);
        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // damping factor
      double damping_factor = this->dampingFactor(w,
                                                  we,
                                                  task->b(),
                                                  task->wa(),
                                                  task->dl(),
                                                  task->du(),
                                                  task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
      }

      return true;
  }

  bool PWTController::setupTask2_5(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                   std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                   const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                                   double w_scale,
                                   double k,
                                   double dt,
                                   double w,
                                   double we){
      if(!this->task2_5_) {
        this->task2_5_ = std::make_shared<prioritized_qp::Task>();
        task = this->task2_5_;
        task->name() = "Task2.5: Contact Force (Break Contact)";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task2_5_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;

      {
        // break contact force constraint
        // 各行は無次元化された上でw_scaleで割られてm,radのオーダにする
        for(size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          contactPoints[i]->desiredForceConstraintForKinematicsConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dwas[i] / w_scale);
          bs.push_back(b / w_scale);
          was.push_back(wa);
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale);
          dus.push_back(du / w_scale);
          wcs.push_back(wc);
        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;

      // damping factor
      double damping_factor = this->dampingFactor(w,
                                                  we,
                                                  task->b(),
                                                  task->wa(),
                                                  task->dl(),
                                                  task->du(),
                                                  task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
      }

      return true;
  }

  bool PWTController::setupTask3(std::shared_ptr<prioritized_qp::Task>& taskHelper, //返り値
                                 std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                                 double w_scale,//次元の大きさを揃え、計算を安定化する
                                 double tau_scale,//次元の大きさを揃え、計算を安定化する
                                 double taumax_weight,//tauに比したtaumaxの重み
                                 double k,
                                 double dt,
                                 double w){
      if(!this->task3Helper_) {
        this->task3Helper_ = std::make_shared<prioritized_qp::Task>();
        taskHelper = this->task3Helper_;
        taskHelper->name() = "Task3Helper: Maximun Joint Torque";
        taskHelper->solver().settings()->resetDefaultSettings();
        taskHelper->solver().settings()->setVerbosity(debug_print_);
        taskHelper->solver().settings()->setWarmStart(true);
        taskHelper->solver().settings()->setMaxIteration(4000);
        taskHelper->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        taskHelper->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        taskHelper->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        taskHelper->toSolve() = false; //最大値を定義するだけなので解かない
      }else{
        taskHelper = this->task3Helper_;
        if(taskHelper->solver().settings()->getSettings()->verbose != debug_print_){
          taskHelper->solver().settings()->setVerbosity(debug_print_);
          taskHelper->solver().clearSolver();
        }
      }

      if(!this->task3_) {
        this->task3_ = std::make_shared<prioritized_qp::Task>();
        task = this->task3_;
        task->name() = "Task3: Contact Force, Joint Torque Reduction";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task3_;
        if(task->solver().settings()->getSettings()->verbose != debug_print_){
          task->solver().settings()->setVerbosity(debug_print_);
          task->solver().clearSolver();
        }
      }

      size_t cols = 0;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()) cols++;
      }

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > AsHelper;
      std::vector<cnoid::VectorXd> bsHelper;
      std::vector<cnoid::VectorXd> wasHelper;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > CsHelper;
      std::vector<cnoid::VectorXd> dlsHelper;
      std::vector<cnoid::VectorXd> dusHelper;
      std::vector<cnoid::VectorXd> wcsHelper;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > A_extsHelper;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > C_extsHelper;

      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
      std::vector<cnoid::VectorXd> bs;
      std::vector<cnoid::VectorXd> was;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
      std::vector<cnoid::VectorXd> dls;
      std::vector<cnoid::VectorXd> dus;
      std::vector<cnoid::VectorXd> wcs;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > A_exts;
      std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > C_exts;

      {
        // contact force reduction
        for(size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          contactPoints[i]->bestEffortForceConstraintForKinematicsConstraint(A,b,wa,C,dl,du,wc);

          As.push_back(A*Dwas[i] / w_scale);
          bs.push_back(b / w_scale);
          was.push_back(wa);
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale);
          dus.push_back(du / w_scale);
          wcs.push_back(wc);

          Eigen::SparseMatrix<double,Eigen::RowMajor> A_ext(A.rows(),1);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C_ext(C.rows(),1);
          A_exts.push_back(A_ext);
          C_exts.push_back(C_ext);

        }
      }

      {
        // joint torque reduction
        for(size_t i=0;i<jointInfos.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          jointInfos[i]->bestEffortJointTorqueConstraint(A,b,wa,C,dl,du,wc);

          const Eigen::SparseMatrix<double, Eigen::RowMajor>& S = jointInfos[i]->torqueSelectMatrix();
          As.push_back(A*S*Dtaua / tau_scale);
          bs.push_back(b / tau_scale);
          was.push_back(wa);
          Cs.push_back(C*S*Dtaua / tau_scale);
          dls.push_back(dl / tau_scale);
          dus.push_back(du / tau_scale);
          wcs.push_back(wc);

          Eigen::SparseMatrix<double,Eigen::RowMajor> A_ext(A.rows(),1);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C_ext(C.rows(),1);
          A_exts.push_back(A_ext);
          C_exts.push_back(C_ext);
        }
      }

      {
        // taumax reduction

        // すでにA b C dの行がm,radの次元にそろえてあるため、ここではそろえる必要はない
        double maximum = 0.0;
        for(size_t i=0;i<jointInfos.size();i++){
          const cnoid::VectorX& b = bs[bs.size()-jointInfos.size()+i];
          for(size_t j=0;j<b.size();j++){
            double e = std::abs(b[j]);
            if(e > maximum) maximum = e;
          }
          const cnoid::VectorX& du = dus[dus.size()-jointInfos.size()+i];
          for(size_t j=0;j<du.size();j++){
            double e = std::abs(std::min(0.0,du[j]));
            if(e > maximum) maximum = e;
          }
          const cnoid::VectorX& dl = dls[dls.size()-jointInfos.size()+i];
          for(size_t j=0;j<dl.size();j++){
            double e = std::abs(std::max(0.0,dl[j]));
            if(e > maximum) maximum = e;
          }
        }

        // define taumax
        for(size_t i=0;i<jointInfos.size();i++){
          {
            const Eigen::SparseMatrix<double,Eigen::RowMajor>& A = As[As.size()-jointInfos.size()+i];
            const cnoid::VectorX& b = bs[bs.size()-jointInfos.size()+i];
            CsHelper.push_back(A);
            Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(A.rows(),1);
            for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = -1.0;
            C_extsHelper.push_back(tmpC_ext);
            cnoid::VectorX tmpdl(b.size());
            for(size_t j=0;j<tmpdl.size();j++) tmpdl[j] = - std::numeric_limits<double>::max();
            dlsHelper.push_back(tmpdl);
            dusHelper.push_back(b - tmpC_ext * maximum);
          }
          {
            const Eigen::SparseMatrix<double,Eigen::RowMajor>& A = As[As.size()-jointInfos.size()+i];
            const cnoid::VectorX& b = bs[bs.size()-jointInfos.size()+i];
            CsHelper.push_back(A);
            Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(A.rows(),1);
            for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = 1.0;
            C_extsHelper.push_back(tmpC_ext);
            cnoid::VectorX tmpdu(b.size());
            for(size_t j=0;j<tmpdu.size();j++) tmpdu[j] = std::numeric_limits<double>::max();
            dusHelper.push_back(tmpdu);
            dlsHelper.push_back(b - tmpC_ext * maximum);
          }
          {
            const Eigen::SparseMatrix<double,Eigen::RowMajor>& C = Cs[Cs.size()-jointInfos.size()+i];
            const cnoid::VectorX& du = dus[dus.size()-jointInfos.size()+i];
            CsHelper.push_back(C);
            Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(C.rows(),1);
            for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = -1.0;
            C_extsHelper.push_back(tmpC_ext);
            cnoid::VectorX tmpdl(du.size());
            for(size_t j=0;j<tmpdl.size();j++) tmpdl[j] = -std::numeric_limits<double>::max();
            dlsHelper.push_back(tmpdl);
            dusHelper.push_back(du - tmpC_ext * maximum);
          }
          {
            const Eigen::SparseMatrix<double,Eigen::RowMajor>& C = Cs[Cs.size()-jointInfos.size()+i];
            const cnoid::VectorX& dl = dls[dls.size()-jointInfos.size()+i];
            CsHelper.push_back(C);
            Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(C.rows(),1);
            for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = 1.0;
            C_extsHelper.push_back(tmpC_ext);
            cnoid::VectorX tmpdu(dl.size());
            for(size_t j=0;j<tmpdu.size();j++) tmpdu[j] = std::numeric_limits<double>::max();
            dusHelper.push_back(tmpdu);
            dlsHelper.push_back(dl - tmpC_ext * maximum);
          }
          wcsHelper.push_back(wcs[wcs.size()-jointInfos.size()+i]);//解かないので使わない
        }

        //reduce taumax
        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A(1,cols);
          cnoid::VectorX b(1);
          cnoid::VectorX wa(1);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C(0,cols);
          cnoid::VectorX dl(0);
          cnoid::VectorX du(0);
          cnoid::VectorX wc(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> A_ext(1,1);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C_ext(0,1);

          A_ext.insert(0,0) = 1.0;
          b[0] = - maximum;
          wa[0] = 1.0 * taumax_weight;

          As.push_back(A);
          bs.push_back(b);
          was.push_back(wa);
          Cs.push_back(C);
          dls.push_back(dl);
          dus.push_back(du);
          wcs.push_back(wc);
          A_exts.push_back(A_ext);
          C_exts.push_back(C_ext);

        }
      }

      task->A().resize(task->A().rows(),cols);
      cnoidbodyutils::appendRow(As, task->A());
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      task->C().resize(task->C().rows(),cols);
      cnoidbodyutils::appendRow(Cs, task->C());
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());
      task->A_ext().resize(task->A_ext().rows(),1);
      cnoidbodyutils::appendRow(A_exts, task->A_ext());
      task->C_ext().resize(task->C_ext().rows(),1);
      cnoidbodyutils::appendRow(C_exts, task->C_ext());

      taskHelper->A().resize(taskHelper->A().rows(),cols);
      cnoidbodyutils::appendRow(AsHelper, taskHelper->A());
      cnoidbodyutils::appendRow(bsHelper, taskHelper->b());
      cnoidbodyutils::appendRow(wasHelper, taskHelper->wa());
      taskHelper->C().resize(taskHelper->C().rows(),cols);
      cnoidbodyutils::appendRow(CsHelper, taskHelper->C());
      cnoidbodyutils::appendRow(dlsHelper, taskHelper->dl());
      cnoidbodyutils::appendRow(dusHelper, taskHelper->du());
      cnoidbodyutils::appendRow(wcsHelper, taskHelper->wc());
      taskHelper->A_ext().resize(taskHelper->A_ext().rows(),1);
      cnoidbodyutils::appendRow(A_extsHelper, taskHelper->A_ext());
      taskHelper->C_ext().resize(taskHelper->C_ext().rows(),1);
      cnoidbodyutils::appendRow(C_extsHelper, taskHelper->C_ext());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;
      // task->A_ext() *= k / dt;
      // task->C_ext() *= k / dt;
      taskHelper->A() *= k / dt;
      taskHelper->C() *= k / dt;
      // taskHelper->A_ext() *= k / dt;
      // taskHelper->C_ext() *= k / dt;

      // damping factor
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = w;
      task->w_ext().resize(1);
      for(size_t i=0;i<task->w_ext().size();i++)task->w_ext()[i] = w;
      taskHelper->w().resize(cols);//解かないので使わない
      for(size_t i=0;i<taskHelper->w().size();i++)taskHelper->w()[i] = w;//解かないので使わない
      taskHelper->w_ext().resize(1);//解かないので使わない
      for(size_t i=0;i<taskHelper->w_ext().size();i++)taskHelper->w_ext()[i] = w;//解かないので使わない

      //id
      task->id_ext() = std::vector<std::string>{"taumax"};
      taskHelper->id_ext() = std::vector<std::string>{"taumax"};

      if(debug_print_){
        std::cerr << taskHelper->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << taskHelper->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << taskHelper->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << taskHelper->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << taskHelper->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << taskHelper->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << taskHelper->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << taskHelper->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << taskHelper->w() << std::endl;
        std::cerr << "A_ext" << std::endl;
        std::cerr << taskHelper->A_ext() << std::endl;
        std::cerr << "C_ext" << std::endl;
        std::cerr << taskHelper->C_ext() << std::endl;
        std::cerr << "w_ext" << std::endl;
        std::cerr << taskHelper->w_ext() << std::endl;
      }

      if(debug_print_){
        std::cerr << task->name() << std::endl;
        std::cerr << "A" << std::endl;
        std::cerr << task->A() << std::endl;
        std::cerr << "b" << std::endl;
        std::cerr << task->b() << std::endl;
        std::cerr << "C" << std::endl;
        std::cerr << task->C() << std::endl;
        std::cerr << "dl" << std::endl;
        std::cerr << task->dl() << std::endl;
        std::cerr << "du" << std::endl;
        std::cerr << task->du() << std::endl;
        std::cerr << "wa" << std::endl;
        std::cerr << task->wa() << std::endl;
        std::cerr << "wc" << std::endl;
        std::cerr << task->wc() << std::endl;
        std::cerr << "w" << std::endl;
        std::cerr << task->w() << std::endl;
        std::cerr << "A_ext" << std::endl;
        std::cerr << task->A_ext() << std::endl;
        std::cerr << "C_ext" << std::endl;
        std::cerr << task->C_ext() << std::endl;
        std::cerr << "w_ext" << std::endl;
        std::cerr << task->w_ext() << std::endl;
      }

      return true;
  }

  double PWTController::dampingFactor(double w,
                                      double we,
                                      const cnoid::VectorX& b,
                                      const cnoid::VectorX& wa,
                                      const cnoid::VectorX& dl,
                                      const cnoid::VectorX& du,
                                      const cnoid::VectorX& wc){
    double e = 0;

    Eigen::SparseMatrix<double,Eigen::RowMajor> Wa(wa.size(),wa.size());
    for(size_t i=0;i<wa.size();i++) Wa.insert(i,i) = wa[i];
    e += b.transpose() * Wa * b;

    Eigen::SparseMatrix<double,Eigen::RowMajor> Wc(wc.size(),wc.size());
    for(size_t i=0;i<wc.size();i++) Wc.insert(i,i) = wc[i];
    cnoid::VectorX dl_e = dl;
    for(size_t i=0;i<dl_e.size();i++) if(dl_e[i]<0) dl_e[i] = 0.0;
    cnoid::VectorX du_e = du;
    for(size_t i=0;i<du_e.size();i++) if(du_e[i]>0) du_e[i] = 0.0;
    e += dl_e.transpose() * Wc * dl_e;
    e += du_e.transpose() * Wc * du_e;
    return w + we * e;
  }

};
