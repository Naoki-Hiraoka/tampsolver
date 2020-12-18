#include <multicontact_controller/PWTController/PWTController.h>

#include <cnoid/EigenUtil>


namespace multicontact_controller {
  // S J dqa = 0 となるSを返す. ? x 6
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPointPWTC::selectMatrixForKinematicsConstraint(){
    return selectMatrixForKinematicsConstraint_;
  }
  // KinematicsConstraint による拘束力の接触維持に必要な制約を返す.
  void ContactPointPWTC::contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    return;
  }
  // KinematicsConstraint による拘束力の目標値を返す。主に接触解除時用
  void ContactPointPWTC::desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    return;
  }
  // KinematicsConstraint による拘束力のベストエフォートタスクを返す。主に負荷低減、安定余裕増大用
  void ContactPointPWTC::bestEffortForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorXd& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorXd& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    return;
  }

  // 位置の目標値を返す。主に遊脚用
  void ContactPointPWTC::desiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    return;
  }

  // 指令関節角度上下限に関する制約を返す.破損防止
  void JointInfo::JointAngleConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    return;
  }

  // 指令関節角速度上下限に関する制約を返す.破損防止
  void JointInfo::JointVelocityConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    return;
  }

  // M \tau によってこのJointの成分を抽出できるM(select matrix). \tauは[numJoints]
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& JointInfo::torqueSelectMatrix(){
    return torqueSelectMatrix_;
  }

  // 関節トルク上下限に関する制約を返す.破損防止
  void JointInfo::JointTorqueConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
    return;
  }

  bool PWTController::calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints, double dt){

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

    // setup tasks
    std::vector<std::shared_ptr<prioritized_qp::Task> > tasks;

    {
      // priority 0
      std::shared_ptr<prioritized_qp::Task> task0;
      if(!this->setupTask0(task0, robot_, jointInfos_, k0_, dt, 0, 1)){
        std::cerr << "setupTask0 failed" << std::endl;
        return false;
      }
      tasks.push_back(task0);
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
                           we1_,
                           0,
                           1)){
        std::cerr << "setupTask1 failed" << std::endl;
        return false;
      }
      tasks.push_back(task1);
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
                           we2_,
                           0,
                           1)){
        std::cerr << "setupTask2 failed" << std::endl;
        return false;
      }
      tasks.push_back(task2);
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
      Xlower.leftCols(6+robot->numJoints()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(Jbal.transpose());
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
                                 double dt,
                                 size_t additional_cols_before,
                                 size_t additional_cols_after){
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

      {
        // joint angle limit
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
        // joint velocity limit
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

      size_t realcols = cols + additional_cols_before + additional_cols_after;
      Eigen::SparseMatrix<double, Eigen::RowMajor> S(cols, realcols);
      for(size_t i=0;i<cols;i++)S.insert(i,i) = 1.0;

      Eigen::SparseMatrix<double, Eigen::RowMajor> A(0,cols);
      cnoidbodyutils::appendRow(As, A);
      task->A() = A * S;
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      Eigen::SparseMatrix<double, Eigen::RowMajor> C(0,cols);
      cnoidbodyutils::appendRow(Cs, C);
      task->C() = C * S;
      cnoidbodyutils::appendRow(dls, task->dl());
      cnoidbodyutils::appendRow(dus, task->du());
      cnoidbodyutils::appendRow(wcs, task->wc());

      // velocity damper
      task->A() *= k / dt;
      task->C() *= k / dt;

      task->w().resize(realcols);//解かないので使わない

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
                                 double we,
                                 size_t additional_cols_before,
                                 size_t additional_cols_after){
      if(!this->task1_) {
        this->task1_ = std::make_shared<prioritized_qp::Task>();
        task = this->task1_;
        task->name() = "Task1: Contact Force, Joint Torque";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task1_;
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
        size_t idx = 0;
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
          was.push_back(wa * std::pow(w_scale,2));
          Cs.push_back(C*Dwas[i] / w_scale);
          dls.push_back(dl / w_scale);
          dus.push_back(du / w_scale);
          wcs.push_back(wc * std::pow(w_scale,2));
          idx++;
        }
      }

      {
        // joint torque limit
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
          was.push_back(wa * std::pow(tau_scale,2));
          Cs.push_back(C*S*Dtaua / tau_scale);
          dls.push_back(dl / tau_scale);
          dus.push_back(du / tau_scale);
          wcs.push_back(wc * std::pow(tau_scale,2));
        }
      }

      size_t realcols = cols + additional_cols_before + additional_cols_after;
      Eigen::SparseMatrix<double, Eigen::RowMajor> S(cols, realcols);
      for(size_t i=0;i<cols;i++)S.insert(i,i) = 1.0;

      Eigen::SparseMatrix<double, Eigen::RowMajor> A(0,cols);
      cnoidbodyutils::appendRow(As, A);
      task->A() = A * S;
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      Eigen::SparseMatrix<double, Eigen::RowMajor> C(0,cols);
      cnoidbodyutils::appendRow(Cs, C);
      task->C() = C * S;
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
      task->w().resize(realcols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

      return true;
  }

  bool PWTController::setupTask2(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                 cnoid::Body* robot,
                                 std::vector<std::shared_ptr<JointInfo> >& jointInfos,
                                 std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,
                                 double dt,
                                 double w,
                                 double we,
                                 size_t additional_cols_before,
                                 size_t additional_cols_after){
      if(!this->task2_) {
        this->task2_ = std::make_shared<prioritized_qp::Task>();
        task = this->task2_;
        task->name() = "Task2: Interacting EndEffector";
        task->solver().settings()->resetDefaultSettings();
        task->solver().settings()->setVerbosity(false);
        task->solver().settings()->setWarmStart(true);
        task->solver().settings()->setMaxIteration(4000);
        task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
        task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
        task->toSolve() = true;
      }else{
        task = this->task2_;
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
        size_t idx = 0;
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
          idx++;
        }
      }

      size_t realcols = cols + additional_cols_before + additional_cols_after;
      Eigen::SparseMatrix<double, Eigen::RowMajor> S(cols, realcols);
      for(size_t i=0;i<cols;i++)S.insert(i,i) = 1.0;

      Eigen::SparseMatrix<double, Eigen::RowMajor> A(0,cols);
      cnoidbodyutils::appendRow(As, A);
      task->A() = A * S;
      cnoidbodyutils::appendRow(bs, task->b());
      cnoidbodyutils::appendRow(was, task->wa());
      Eigen::SparseMatrix<double, Eigen::RowMajor> C(0,cols);
      cnoidbodyutils::appendRow(Cs, C);
      task->C() = C * S;
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
      task->w().resize(realcols);
      for(size_t i=0;i<task->w().size();i++)task->w()[i] = damping_factor;

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
