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
  // 各行は/iter次元化されている
  void ContactPointPWTC::contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    if(this->state() == "CONTACT"){
      this->contact()->getContactConstraint(A,b,wa,C,dl,du,wc);
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
      b *= dt_/k_;// velocity damper
      dl *= dt_/k_;// velocity damper
      du *= dt_/k_;// velocity damper
    }else if(this->state() == "TOWARD_BREAK_CONTACT"){
      this->contact()->getContactConstraint(A,b,wa,C,dl,du,wc,true);//allow_break_contact
      b -= A * selectMatrixForKinematicsConstraint() * F_;
      dl -= C * selectMatrixForKinematicsConstraint() * F_;
      du -= C * selectMatrixForKinematicsConstraint() * F_;
      b *= dt_/k_;// velocity damper
      for(size_t i=0;i<dl.size();i++){
        if(dl[i]>0.0) dl[i] *= dt_/k_;// velocity damper
        else dl[i] *= 1.0;// 接触解除速度を早める
      }
      for(size_t i=0;i<du.size();i++){
        if(du[i]<0.0) du[i] *= dt_/k_;// velocity damper
        else du[i] *= 1.0;// 接触解除速度を早める
      }
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
      this->contact()->getBreakContactMotionConstraint(A,b,wa,C,dl,du,wc);
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
  // 各行はm / iter, rad / iter
  void ContactPointPWTC::desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A_A, cnoid::VectorX& b_A, cnoid::VectorX& wa_A, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_A, cnoid::VectorX& dl_A, cnoid::VectorXd& du_A, cnoid::VectorX& wc_A,
                                                   Eigen::SparseMatrix<double,Eigen::RowMajor>& A_B, cnoid::VectorX& b_B, cnoid::VectorX& wa_B, Eigen::SparseMatrix<double,Eigen::RowMajor>& C_B, cnoid::VectorX& dl_B, cnoid::VectorXd& du_B, cnoid::VectorX& wc_B){
    if(this->state() == "AIR" || this->state() == "NEAR_CONTACT"){
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local_A;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local_A;
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local_B;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local_B;
      this->interaction()->desiredPositionConstraint(A_local_A,b_A,wa_A,C_local_A,dl_A,du_A,wc_A,
                                                     A_local_B,b_B,wa_B,C_local_B,dl_B,du_B,wc_B);
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& J = this->calcJacobian();//world系,contactpoint周り
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Rinv = this->calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
      A_A = A_local_A * Rinv * J;
      C_A = C_local_A * Rinv * J;
      A_B = A_local_B * Rinv * J;
      C_B = C_local_B * Rinv * J;
    } else if(this->state() == "TOWARD_MAKE_CONTACT"){
      cnoid::Vector6 contactDirection = this->contact()->selectMatrix().transpose() * this->contact()->contactDirection();
      if(contactDirection.head<3>().norm() > 0){
        contactDirection.head<3>() = contactDirection.head<3>().normalized() * std::min(this->interaction()->v_limit(),this->contact()->contact_v_limit()) * this->interaction()->dt();
        this->interaction()->T_ref().translation() += contactDirection.head<3>();
      }
      if(contactDirection.tail<3>().norm() > 0){
        contactDirection.tail<3>() = contactDirection.tail<3>().normalized() * std::min(this->interaction()->w_limit(),this->contact()->contact_v_limit()) * this->interaction()->dt(); //TODO v_limitのまま
        this->interaction()->T_ref().linear() = (this->interaction()->T_ref().linear() * cnoid::AngleAxis(contactDirection.tail<3>().norm(),contactDirection.tail<3>().normalized())).eval();
      }
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local_A;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local_A;
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_local_B;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_local_B;
      this->interaction()->desiredPositionConstraint(A_local_A,b_A,wa_A,C_local_A,dl_A,du_A,wc_A,
                                                     A_local_B,b_B,wa_B,C_local_B,dl_B,du_B,wc_B);
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& J = this->calcJacobian();//world系,contactpoint周り
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Rinv = this->calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
      A_A = A_local_A * Rinv * J;
      C_A = C_local_A * Rinv * J;
      A_B = A_local_B * Rinv * J;
      C_B = C_local_B * Rinv * J;
    }else{
      A_A.resize(0,6+this->parent()->body()->numJoints());
      b_A.resize(0);
      wa_A.resize(0);
      C_A.resize(0,6+this->parent()->body()->numJoints());
      dl_A.resize(0);
      du_A.resize(0);
      wc_A.resize(0);
      A_B.resize(0,6+this->parent()->body()->numJoints());
      b_B.resize(0);
      wa_B.resize(0);
      C_B.resize(0,6+this->parent()->body()->numJoints());
      dl_B.resize(0);
      du_B.resize(0);
      wc_B.resize(0);
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

  bool PWTController::calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                     std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                     std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& pclCollisions,
                                     double dt){

    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Js;Js.reserve(contactPoints.size());//contactpoint系,contactpoint周り
    for(size_t i=0;i<contactPoints.size();i++) Js.push_back(contactPoints[i]->calcRinv() * contactPoints[i]->calcJacobian());

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
      std::shared_ptr<prioritized_qp::Task> task2_A;
      std::shared_ptr<prioritized_qp::Task> task2_B;
      if(!this->setupTask2(task2_A,
                           task2_B,
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
      tasks.push_back(task2_A);
      tasks.push_back(task2_B);
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
                           w_weight3_,
                           tau_weight3_,
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
    bool solved = prioritized_qp::solve(tasks,result, debug_print_);
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

    return true;
  }

  bool PWTController::calcPWTJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,//返り値
                                      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,//返り値
                                      Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,//返り値
                                      cnoid::Body* robot,
                                      std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                      cnoidbodyutils::TorqueJacobianCalculator& torqueJacobianCalculator,
                                      std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Js,
                                      double sv_ratio){

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dg = torqueJacobianCalculator.calcDg();

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& DJw = torqueJacobianCalculator.calcDJw(contactPoints); // F = 0 のcontactpointをここで入れるのは計算コストの無駄かもしれない

    for(size_t i=0;i<robot->numJoints();i++){
      Ka_.coeffRef(6+i,6+i) = jointInfos[i]->pgain();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> Jbal(0,6+robot->numJoints());
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Jbals;
    for(size_t i=0;i<contactPoints.size();i++) {
      Jbals.push_back(contactPoints[i]->selectMatrixForKinematicsConstraint() * Js[i]);
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
                                 std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                 double k,
                                 double dt){
      if(!this->task0_) {
        this->task0_ = std::make_shared<prioritized_qp::Task>();
        task = this->task0_;
        task->name() = "Task0: Joint Limit";
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
            Eigen::SparseMatrix<double, Eigen::RowMajor> S(1,cols); S.insert(0,idx) = 1.0;
            Eigen::SparseMatrix<double,Eigen::RowMajor> A;
            cnoid::VectorX b;
            cnoid::VectorX wa;
            Eigen::SparseMatrix<double,Eigen::RowMajor> C;
            cnoid::VectorX dl;
            cnoid::VectorX du;
            cnoid::VectorX wc;
            jointInfos[i]->JointAngleConstraint(A,b,wa,C,dl,du,wc,true);//command angleに対して
            As.push_back(A*S);
            bs.push_back(b * dt/k); // velocity damper
            was.push_back(wa);
            Cs.push_back(C*S);
            dls.push_back(dl * dt/k); // velocity damper
            dus.push_back(du * dt/k); // velocity damper
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

      return true;
  }

  bool PWTController::setupTask0_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
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
      task->b() *= dt / k;
      task->dl() *= dt / k;
      task->du() *= dt / k;

      // damping factor
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            task->b(),
                                                            task->wa(),
                                                            task->dl(),
                                                            task->du(),
                                                            task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      return true;
  }


  bool PWTController::setupTask1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
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
          contactPoints[i]->k() = k;

          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          cnoid::VectorX b;
          cnoid::VectorX wa;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          cnoid::VectorX dl;
          cnoid::VectorX du;
          cnoid::VectorX wc;
          contactPoints[i]->contactForceConstraintForKinematicsConstraint(A,b,wa,C,dl,du,wc);
          As.push_back(A*Dwas[i] / w_scale);
          bs.push_back(b / w_scale); // velocity damper
          was.push_back(wa);
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale); // velocity damper
          dus.push_back(du / w_scale); // velocity damper
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
          bs.push_back(b *dt/k / tau_scale); // velocity damper
          was.push_back(wa);
          Cs.push_back(C*S*Dtaua / tau_scale);
          dls.push_back(dl *dt/k / tau_scale); // velocity damper
          dus.push_back(du *dt/k / tau_scale); // velocity damper
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
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            task->b(),
                                                            task->wa(),
                                                            task->dl(),
                                                            task->du(),
                                                            task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      return true;
  }

  bool PWTController::setupTask1_1(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
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
      task->b() *= dt / k;
      task->dl() *= dt / k;
      task->du() *= dt / k;

      // damping factor
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            task->b(),
                                                            task->wa(),
                                                            task->dl(),
                                                            task->du(),
                                                            task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      return true;
  }

  bool PWTController::setupTask2(std::shared_ptr<prioritized_qp::Task>& taskA, //返り値
                                 std::shared_ptr<prioritized_qp::Task>& taskB, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                                 double dt,
                                 double w,
                                 double we){
    if(!this->task2_A_) {
      this->task2_A_ = std::make_shared<prioritized_qp::Task>();
      taskA = this->task2_A_;
      taskA->name() = "Task2_A: Interacting EndEffector";
      taskA->solver().settings()->resetDefaultSettings();
      taskA->solver().settings()->setVerbosity(debug_print_);
      taskA->solver().settings()->setWarmStart(true);
      taskA->solver().settings()->setMaxIteration(4000);
      taskA->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      taskA->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      taskA->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      taskA->toSolve() = true;
    }else{
      taskA = this->task2_A_;
      if(taskA->solver().settings()->getSettings()->verbose != debug_print_){
        taskA->solver().settings()->setVerbosity(debug_print_);
        taskA->solver().clearSolver();
      }
    }

    if(!this->task2_B_) {
      this->task2_B_ = std::make_shared<prioritized_qp::Task>();
      taskB = this->task2_B_;
      taskB->name() = "Task2_B: Interacting EndEffector";
      taskB->solver().settings()->resetDefaultSettings();
      taskB->solver().settings()->setVerbosity(debug_print_);
      taskB->solver().settings()->setWarmStart(true);
      taskB->solver().settings()->setMaxIteration(4000);
      taskB->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      taskB->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      taskB->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      taskB->toSolve() = true;
    }else{
      taskB = this->task2_B_;
      if(taskB->solver().settings()->getSettings()->verbose != debug_print_){
        taskB->solver().settings()->setVerbosity(debug_print_);
        taskB->solver().clearSolver();
      }
    }

    size_t cols = 0;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }

    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > A_As;
    std::vector<cnoid::VectorXd> b_As;
    std::vector<cnoid::VectorXd> wa_As;
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > C_As;
    std::vector<cnoid::VectorXd> dl_As;
    std::vector<cnoid::VectorXd> du_As;
    std::vector<cnoid::VectorXd> wc_As;

    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > A_Bs;
    std::vector<cnoid::VectorXd> b_Bs;
    std::vector<cnoid::VectorXd> wa_Bs;
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > C_Bs;
    std::vector<cnoid::VectorXd> dl_Bs;
    std::vector<cnoid::VectorXd> du_Bs;
    std::vector<cnoid::VectorXd> wc_Bs;

    {
      // interaction end_effector
      // 各行はm,radのオーダ
      for(size_t i=0;i<contactPoints.size();i++){
        Eigen::SparseMatrix<double,Eigen::RowMajor> A_A;
        cnoid::VectorX b_A;
        cnoid::VectorX wa_A;
        Eigen::SparseMatrix<double,Eigen::RowMajor> C_A;
        cnoid::VectorX dl_A;
        cnoid::VectorX du_A;
        cnoid::VectorX wc_A;
        Eigen::SparseMatrix<double,Eigen::RowMajor> A_B;
        cnoid::VectorX b_B;
        cnoid::VectorX wa_B;
        Eigen::SparseMatrix<double,Eigen::RowMajor> C_B;
        cnoid::VectorX dl_B;
        cnoid::VectorX du_B;
        cnoid::VectorX wc_B;
        contactPoints[i]->desiredPositionConstraint(A_A,b_A,wa_A,C_A,dl_A,du_A,wc_A,
                                                    A_B,b_B,wa_B,C_B,dl_B,du_B,wc_B);
        A_As.push_back(A_A*Dqa);
        b_As.push_back(b_A);
        wa_As.push_back(wa_A);
        C_As.push_back(C_A*Dqa);
        dl_As.push_back(dl_A);
        du_As.push_back(du_A);
        wc_As.push_back(wc_A);
        A_Bs.push_back(A_B*Dqa);
        b_Bs.push_back(b_B);
        wa_Bs.push_back(wa_B);
        C_Bs.push_back(C_B*Dqa);
        dl_Bs.push_back(dl_B);
        du_Bs.push_back(du_B);
        wc_Bs.push_back(wc_B);
      }
    }

    {
      taskA->A().resize(taskA->A().rows(),cols);
      cnoidbodyutils::appendRow(A_As, taskA->A());
      cnoidbodyutils::appendRow(b_As, taskA->b());
      cnoidbodyutils::appendRow(wa_As, taskA->wa());
      taskA->C().resize(taskA->C().rows(),cols);
      cnoidbodyutils::appendRow(C_As, taskA->C());
      cnoidbodyutils::appendRow(dl_As, taskA->dl());
      cnoidbodyutils::appendRow(du_As, taskA->du());
      cnoidbodyutils::appendRow(wc_As, taskA->wc());

      // damping factor
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            taskA->b(),
                                                            taskA->wa(),
                                                            taskA->dl(),
                                                            taskA->du(),
                                                            taskA->wc());
      taskA->w().resize(cols);
      for(size_t i=0;i<taskA->w().size();i++)taskA->w()[i] = damping_factor;
    }

    {
      taskB->A().resize(taskB->A().rows(),cols);
      cnoidbodyutils::appendRow(A_Bs, taskB->A());
      cnoidbodyutils::appendRow(b_Bs, taskB->b());
      cnoidbodyutils::appendRow(wa_Bs, taskB->wa());
      taskB->C().resize(taskB->C().rows(),cols);
      cnoidbodyutils::appendRow(C_Bs, taskB->C());
      cnoidbodyutils::appendRow(dl_Bs, taskB->dl());
      cnoidbodyutils::appendRow(du_Bs, taskB->du());
      cnoidbodyutils::appendRow(wc_Bs, taskB->wc());

      // damping factor
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            taskB->b(),
                                                            taskB->wa(),
                                                            taskB->dl(),
                                                            taskB->du(),
                                                            taskB->wc());
      taskB->w().resize(cols);
      for(size_t i=0;i<taskB->w().size();i++)taskB->w()[i] = damping_factor;

    }


    return true;
  }

  bool PWTController::setupTask2_5(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                   cnoid::Body* robot,
                                   std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                   std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                   const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                                   double w_scale,
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
        // 各行は/iter次元でw_scaleで割られてm,radのオーダにする
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

      // damping factor
      double damping_factor = cnoidbodyutils::dampingFactor(w,
                                                            we,
                                                            task->b(),
                                                            task->wa(),
                                                            task->dl(),
                                                            task->du(),
                                                            task->wc());
      task->w().resize(cols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      return true;
  }

  bool PWTController::setupTask3(std::shared_ptr<prioritized_qp::Task>& taskHelper, //返り値
                                 std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> >& Dwas,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,
                                 double w_scale,//次元の大きさを揃え、計算を安定化する
                                 double tau_scale,//次元の大きさを揃え、計算を安定化する
                                 double w_weight,
                                 double tau_weight,
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
          was.push_back(wa * w_weight);
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale);
          dus.push_back(du / w_scale);
          wcs.push_back(wc * w_weight);

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
          was.push_back(wa * tau_weight);
          Cs.push_back(C*S*Dtaua / tau_scale);
          dls.push_back(dl / tau_scale);
          dus.push_back(du / tau_scale);
          wcs.push_back(wc * tau_weight);

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
        cnoidbodyutils::defineMaximumError(As,bs,Cs,dls,dus,
                                           CsHelper,dlsHelper,dusHelper,wcsHelper,C_extsHelper,maximum,
                                           As.size()-jointInfos.size(),As.size(),
                                           Cs.size()-jointInfos.size(),Cs.size());

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
      task->b() *= dt / k;
      task->dl() *= dt / k;
      task->du() *= dt / k;
      // velocity damper
      taskHelper->b() *= dt / k;
      taskHelper->dl() *= dt / k;
      taskHelper->du() *= dt / k;

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

      return true;
  }

};
