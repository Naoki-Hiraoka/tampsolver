#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityChecker.h>

#include <cnoid/EigenUtil>
#include <cnoid/Jacobian>
#include <static_equilibuim_test/StaticEquilibuimTest.h>

namespace multicontact_controller {
  // S J dqa = 0 となるSを返す. ? x 6
  const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPointCBAC::selectMatrixForKinematicsConstraint(){
    if(this->state() == "CONTACT" || this->state() == "TOWARD_BREAK_CONTACT"){
      return this->contact()->selectMatrix();
    }else{
      selectMatrixForKinematicsConstraint_.resize(0,6);
      return selectMatrixForKinematicsConstraint_;
    }
  }

  // KinematicsConstraint による拘束力の接触維持に必要な制約を返す.
  // 各行は無次元化されている. selectMatrixForKinematicsConstraintの次元
  void ContactPointCBAC::contactForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorX& du, cnoid::VectorX& wc){
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
  // 各行は無次元化されている. selectMatrixForKinematicsConstraintの次元
  void ContactPointCBAC::desiredForceConstraintForKinematicsConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    this->contact()->getBreakContactConstraint(A,b,wa,C,dl,du,wc);
    b -= A * selectMatrixForKinematicsConstraint() * F_;
    dl -= C * selectMatrixForKinematicsConstraint() * F_;
    du -= C * selectMatrixForKinematicsConstraint() * F_;
    return;
  }

  // 支持脚の場合位置の目標値を返す。 colは[root6dof + numJoint]
  // T_actが必要
  // rad m / iter
  void ContactPointCBAC::desiredPositionConstraintBal(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(this->state() == "CONTACT" || this->state() == "TOWARD_BREAK_CONTACT"){
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& J = this->calcJacobian();//world系,contactpoint周り
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Rinv = this->calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& S = this->contact()->selectMatrix();
      A = S * Rinv * J;
      b = this->contact()->calcError(this->parent()->T()*this->T_local(),this->T_act());
      wa.resize(b.size());
      for(size_t i=0;i<wa.size();i++){
        wa[i] = 1.0;
      }
      C.resize(0,6+this->parent()->body()->numJoints());
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }else{
      A.resize(0,6+this->parent()->body()->numJoints());
      b.resize(0);
      wa.resize(0);
      C.resize(0,6+this->parent()->body()->numJoints());
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }
  }

  // 遊脚の場合位置の目標値を返す。 colは[root6dof + numJoint]
  // T_actが必要
  // rad m / iter
  void ContactPointCBAC::desiredPositionConstraintInt(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    if(this->state() == "AIR" ||
       this->state() == "NEAR_CONTACT" ||
       this->state() == "TOWARD_MAKE_CONTACT"){
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& J = this->calcJacobian();//world系,contactpoint周り
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& Rinv = this->calcRinv();//calcJacobianの左から掛けるとcontactpoint系,contactpoint周りになる
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& S = this->contact()->selectMatrix();
      A = S * Rinv * J;
      b = this->contact()->calcError(this->parent()->T()*this->T_local(),this->T_act());
      wa.resize(b.size());
      for(size_t i=0;i<wa.size();i++){
        wa[i] = 1.0;
      }
      C.resize(0,6+this->parent()->body()->numJoints());
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }else{
      A.resize(0,6+this->parent()->body()->numJoints());
      b.resize(0);
      wa.resize(0);
      C.resize(0,6+this->parent()->body()->numJoints());
      dl.resize(0);
      du.resize(0);
      wc.resize(0);
    }
  }

  // 現在のcontactPointsの状態でbreakContactPointのcontactをbreakできるかどうかを調べ、重心位置の余裕を返す
  bool ContactBreakAbilityChecker::Check(double& margin, //返り値
                                         std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
                                         std::shared_ptr<ContactPointCBAC>& breakContactPoint,
                                         bool fixInteraction,
                                         std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                         std::shared_ptr<PCLCollisionDetector>& pCLCollisionDetector
                                         ){
    robot_->calcForwardKinematics();
    robot_->calcCenterOfMass();
    cnoid::Vector2 currentCenterOfMass = robot_->centerOfMass().head(2);
    // currentCenterOfMassからの変化に対する制約
    Eigen::SparseMatrix<double,Eigen::RowMajor> SCFR_M;
    cnoid::VectorXd SCFR_l;
    cnoid::VectorX SCFR_u;
    this->calcSCFR(contactPoints,nullptr,SCFR_M,SCFR_l,SCFR_u);
    Eigen::SparseMatrix<double,Eigen::RowMajor> SCFR_M_break;
    cnoid::VectorXd SCFR_l_break;
    cnoid::VectorX SCFR_u_break;
    this->calcSCFR(contactPoints,breakContactPoint,SCFR_M_break,SCFR_l_break,SCFR_u_break);

    for(size_t i=0;i<contactPoints.size();i++){
      contactPoints[i]->T_act() = contactPoints[i]->parent()->T() * contactPoints[i]->T_local();
    }

    for(size_t i=0;i<loopNum_;i++){
      // setup tasks
      std::vector<std::shared_ptr<prioritized_qp::Task> > tasks; // 変数は[rootlink + controllablejoints]

      {
        // priority 0
        // joint limit
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupJointLimitTask(task,
                                      robot_,
                                      jointInfos_,
                                      w_JointLimit_)){
          std::cerr << "setupJointLimitTask failed" << std::endl;
          return false;
        }
        tasks.push_back(task);
      }

      {
        // priority 1
        // self collision
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupSelfCollisionTask(task,
                                         robot_,
                                         jointInfos_,
                                         selfCollisions,
                                         tolerance_SelfCollision_,
                                         w_SelfCollision_,
                                         we_SelfCollision_)){
          std::cerr << "setupSelfCollisionTask failed" << std::endl;
          return false;
        }
        tasks.push_back(task);
      }

      {
        // priority 2
        // eef position
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupPositionTask(task,
                                    robot_,
                                    jointInfos_,
                                    contactPoints,
                                    fixInteraction,
                                    w_Position_,
                                    we_Position_)){
          std::cerr << "setupPositionTask failed" << std::endl;
          return false;
        }
        tasks.push_back(task);
      }

      {
        // priority 3
        // SCFR
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupSCFRTask(task,
                                robot_,
                                jointInfos_,
                                SCFR_M,
                                SCFR_l,
                                SCFR_u,
                                w_SCFR_,
                                we_SCFR_)){
          std::cerr << "setupSCFRTask failed" << std::endl;
          return false;
        }
        tasks.push_back(task);
      }

      {
        // priority 4
        // pcl collision
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupPCLCollisionTask(task,
                                        robot_,
                                        jointInfos_,
                                        pCLCollisionDetector,
                                        tolerance_PCLCollision_,
                                        w_PCLCollision_,
                                        we_PCLCollision_)){
          std::cerr << "setupPCLCollisionTask failed" << std::endl;
          return false;
        }
        tasks.push_back(task);
      }

      {
        // priority 5
        // SCFR
        std::shared_ptr<prioritized_qp::Task> taskHelper;
        std::shared_ptr<prioritized_qp::Task> task;
        if(!this->setupSCFRBreakTask(taskHelper,
                                     task,
                                     robot_,
                                     jointInfos_,
                                     SCFR_M_break,
                                     SCFR_l_break,
                                     SCFR_u_break,
                                     w_SCFR_break_)){
          std::cerr << "setupSCFRBreakTask failed" << std::endl;
          return false;
        }
        tasks.push_back(taskHelper);
        tasks.push_back(task);
      }

      // Solve
      cnoid::VectorX result;
      bool solved = prioritized_qp::solve(tasks,result, debug_print_);
      if(!solved) {
        std::cerr << "prioritized_qp::solve failed" << std::endl;
        return false;
      }

      //reflect result
      {
        robot_->rootLink()->p() += result.segment<3>(0);
        if(result.segment<3>(3).norm() != 0.0){
          cnoid::Matrix3 dR = cnoid::Matrix3(cnoid::AngleAxis(result.segment<3>(3).norm(), result.segment<3>(3).normalized()));
          robot_->rootLink()->R() = dR * robot_->rootLink()->R();
        }

        size_t idx = 0;
        for(size_t i=0;i<jointInfos_.size();i++){
          if(jointInfos_[i]->controllable()){
            jointInfos_[i]->joint()->q() += result[6+idx];
            idx++;
          }
        }
      }

      robot_->calcForwardKinematics();
      robot_->calcCenterOfMass();
    }

    return true;
  }

  // メンバ変数はrobot_しか使わない
  bool ContactBreakAbilityChecker::calcSCFR(std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
                                            const std::shared_ptr<ContactPointCBAC>& breakContactPoint,
                                            Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,//返り値
                                            cnoid::VectorXd& SCFR_l,//返り値
                                            cnoid::VectorX& SCFR_u,//返り値
                                            double g
                                            ){
    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd b;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C;
    Eigen::VectorXd dl;
    Eigen::VectorXd du;
    {
      // compute A, b, C, dl, du
      // x = [dpx dpy dw1 dw2 ...]^T
      // wはエンドエフェクタ座標系．エンドエフェクタまわり. サイズはselectMatrixForKinematicsConstraint

      // Grasp Matrix Gx = h
      // 現在のrootLinkの位置まわりのつりあい. 座標軸はworld系の向き
      Eigen::SparseMatrix<double,Eigen::RowMajor> G;
      Eigen::VectorXd h = Eigen::VectorXd::Zero(6);
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::ColMajor> > Gs;

        {
          Eigen::SparseMatrix<double,Eigen::ColMajor> G01(6,2);
          G01.insert(3,1) = -this->robot_->mass()*g;
          G01.insert(4,0) = this->robot_->mass()*g;
          Gs.push_back(G01);
        }


        for(size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::ColMajor> GraspMatrix(6,6);
          {
            const cnoid::Position pos = contactPoints[i]->parent()->T() * contactPoints[i]->T_local();
            const cnoid::Matrix3& R = pos.linear();
            const cnoid::Matrix3& p_x_R = cnoid::hat(pos.translation()) * R;

            std::vector<Eigen::Triplet<double> > G_tripletList;
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(row,col,R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                G_tripletList.push_back(Eigen::Triplet<double>(3+row,col,p_x_R(row,col)));
              }
            }
            for(size_t row=0;row<3;row++){
              for(size_t col=0;col<3;col++){
                  G_tripletList.push_back(Eigen::Triplet<double>(3+row,3+col,R(row,col)));
              }
            }
            GraspMatrix.setFromTriplets(G_tripletList.begin(), G_tripletList.end());
          }
          Gs.push_back(GraspMatrix * contactPoints[i]->selectMatrixForKinematicsConstraint().transpose());
        }

        Eigen::SparseMatrix<double,Eigen::ColMajor> G_ColMajor;
        cnoidbodyutils::appendCol(Gs,G_ColMajor);
        G = G_ColMajor;
      }

      //接触力制約
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_contact;
      Eigen::VectorXd b_contact;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_contact;
      Eigen::VectorXd dl_contact;
      Eigen::VectorXd du_contact;
      {
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As;
        std::vector<Eigen::VectorXd> bs;
        std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs;
        std::vector<Eigen::VectorXd> dls;
        std::vector<Eigen::VectorXd> dus;

        {
          Eigen::SparseMatrix<double,Eigen::RowMajor> A01(0,2);
          Eigen::VectorXd b01(0);
          Eigen::SparseMatrix<double,Eigen::RowMajor> C01(0,2);
          Eigen::VectorXd dl01(0);
          Eigen::VectorXd du01(0);
          As.push_back(A01);
          bs.push_back(b01);
          Cs.push_back(C01);
          dls.push_back(dl01);
          dus.push_back(du01);
        }

        for (size_t i=0;i<contactPoints.size();i++){
          Eigen::SparseMatrix<double,Eigen::RowMajor> A;
          Eigen::VectorXd b;
          Eigen::SparseMatrix<double,Eigen::RowMajor> C;
          Eigen::VectorXd dl;
          Eigen::VectorXd du;

          {
            std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As_i;
            std::vector<Eigen::VectorXd> bs_i;
            std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs_i;
            std::vector<Eigen::VectorXd> dls_i;
            std::vector<Eigen::VectorXd> dus_i;
            {
              Eigen::SparseMatrix<double,Eigen::RowMajor> A_i;
              Eigen::VectorXd b_i;
              Eigen::SparseMatrix<double,Eigen::RowMajor> C_i;
              Eigen::VectorXd dl_i;
              Eigen::VectorXd du_i;
              Eigen::VectorXd wa_tmp, wc_tmp;
              contactPoints[i]->contactForceConstraintForKinematicsConstraint(A_i,b_i,wa_tmp,C_i,dl_i,du_i,wc_tmp);
              As_i.push_back(A_i);
              bs_i.push_back(b_i);
              Cs_i.push_back(C_i);
              dls_i.push_back(dl_i);
              dus_i.push_back(du_i);
            }
            if(contactPoints[i] == breakContactPoint) {
              Eigen::SparseMatrix<double,Eigen::RowMajor> A_i;
              Eigen::VectorXd b_i;
              Eigen::SparseMatrix<double,Eigen::RowMajor> C_i;
              Eigen::VectorXd dl_i;
              Eigen::VectorXd du_i;
              Eigen::VectorXd wa_tmp, wc_tmp;
              contactPoints[i]->desiredForceConstraintForKinematicsConstraint(A_i,b_i,wa_tmp,C_i,dl_i,du_i,wc_tmp);
              As_i.push_back(A_i);
              bs_i.push_back(b_i);
              Cs_i.push_back(C_i);
              dls_i.push_back(dl_i);
              dus_i.push_back(du_i);
            }
            cnoidbodyutils::appendRow(As_i,A);
            cnoidbodyutils::appendRow(bs_i,b);
            cnoidbodyutils::appendRow(Cs_i,C);
            cnoidbodyutils::appendRow(dls_i,dl);
            cnoidbodyutils::appendRow(dus_i,du);
          }
          As.push_back(A);
          bs.push_back(b);
          Cs.push_back(C);
          dls.push_back(dl);
          dus.push_back(du);
        }
        cnoidbodyutils::appendDiag(As,A_contact);
        cnoidbodyutils::appendRow(bs,b_contact);
        cnoidbodyutils::appendDiag(Cs,C_contact);
        cnoidbodyutils::appendRow(dls,dl_contact);
        cnoidbodyutils::appendRow(dus,du_contact);
      }

      A = Eigen::SparseMatrix<double,Eigen::RowMajor>(G.rows()+A_contact.rows(),G.cols());
      A.topRows(G.rows()) = G;
      A.bottomRows(A_contact.rows()) = A_contact;
      b = Eigen::VectorXd(h.size()+b_contact.size());
      b.head(h.rows()) = h;
      b.tail(b_contact.rows()) = b_contact;
      C = C_contact;
      dl = dl_contact;
      du = du_contact;
    }

    if(!static_equilibuim_test::calcProjection(A,b,C,dl,du,SCFR_M,SCFR_l,SCFR_u,debug_print_)){
      std::cerr << "[ContactBreakAbilityChecker::calcSCFR] projection failed" << std::endl;
      return false;
    }

    // 各要素を正規化
    for(size_t i=0;i<SCFR_M.rows();i++){
      double norm = SCFR_M.row(i).norm();
      if(norm == 0) continue;
      SCFR_M.coeffRef(i,0) /= norm;
      SCFR_M.coeffRef(i,1) /= norm;
      SCFR_l[i] /= norm;
      SCFR_u[i] /= norm;
    }

    return true;
  }


  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupJointLimitTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                       cnoid::Body* robot,
                                                       std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                       double w){
    if(!this->jointLimitTask_) {
      this->jointLimitTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->jointLimitTask_;
      task->name() = "JointLimitTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->jointLimitTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 6;
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
      // joint angle limit.
      size_t idx = 6;
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

    task->A().resize(task->A().rows(),cols);
    cnoidbodyutils::appendRow(As, task->A());
    cnoidbodyutils::appendRow(bs, task->b());
    cnoidbodyutils::appendRow(was, task->wa());
    task->C().resize(task->C().rows(),cols);
    cnoidbodyutils::appendRow(Cs, task->C());
    cnoidbodyutils::appendRow(dls, task->dl());
    cnoidbodyutils::appendRow(dus, task->du());
    cnoidbodyutils::appendRow(wcs, task->wc());

    task->w().resize(cols);
    for(size_t i=0;i<task->w().size();i++) task->w()[i] = w;

    return true;
  }

  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupSelfCollisionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                          cnoid::Body* robot,
                                                          std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                          std::vector<std::shared_ptr<cnoidbodyutils::Collision> >& selfCollisions,
                                                          double tolerance,
                                                          double w,
                                                          double we){
    if(!this->selfCollisionTask_) {
      this->selfCollisionTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->selfCollisionTask_;
      task->name() = "SelfCollisionTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->selfCollisionTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 6;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }

    Eigen::SparseMatrix<double, Eigen::RowMajor> S(6+robot->numJoints(),cols);
    {
      for(size_t i=0;i<6;i++) S.insert(i,i) = 1.0;
      size_t idx = 6;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()){
          S.insert(6+i,idx) = 1.0;
          idx++;
        }
      }
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
      // TODO アップデートされない
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
        As.push_back(A*S);
        bs.push_back(b);
        was.push_back(wa);
        Cs.push_back(C*S);
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

  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupPositionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                     cnoid::Body* robot,
                                                     std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                     std::vector<std::shared_ptr<ContactPointCBAC> >& contactPoints,
                                                     bool fixInteraction,
                                                     double w,
                                                     double we){
    if(!this->positionTask_) {
      this->positionTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->positionTask_;
      task->name() = "PositionTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->positionTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 0;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> S(6+robot->numJoints(),cols);
    {
      for(size_t i=0;i<6;i++) S.insert(i,i) = 1.0;
      size_t idx = 6;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()){
          S.insert(6+i,idx) = 1.0;
          idx++;
        }
      }
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
        contactPoints[i]->desiredPositionConstraintBal(A,b,wa,C,dl,du,wc);
        As.push_back(A*S);
        bs.push_back(b);
        was.push_back(wa);
        Cs.push_back(C*S);
        dls.push_back(dl);
        dus.push_back(du);
        wcs.push_back(wc);

        if(fixInteraction){
          contactPoints[i]->desiredPositionConstraintInt(A,b,wa,C,dl,du,wc);
          As.push_back(A*S);
          bs.push_back(b);
          was.push_back(wa);
          Cs.push_back(C*S);
          dls.push_back(dl);
          dus.push_back(du);
          wcs.push_back(wc);
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

  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupSCFRTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                 cnoid::Body* robot,
                                                 std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                 const Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,
                                                 const cnoid::VectorXd& SCFR_l,
                                                 const cnoid::VectorX& SCFR_u,
                                                 double w,
                                                 double we){
    if(!this->sCFRTask_) {
      this->sCFRTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->sCFRTask_;
      task->name() = "SCFRTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->sCFRTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 0;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> S(6+robot->numJoints(),cols);
    {
      for(size_t i=0;i<6;i++) S.insert(i,i) = 1.0;
      size_t idx = 6;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()){
          S.insert(6+i,idx) = 1.0;
          idx++;
        }
      }
    }

    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
    std::vector<cnoid::VectorXd> bs;
    std::vector<cnoid::VectorXd> was;
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorXd> dls;
    std::vector<cnoid::VectorXd> dus;
    std::vector<cnoid::VectorXd> wcs;

    {
      // 重心変位
      // 各行はm,radのオーダ
      // calc CM jacobian
      Eigen::SparseMatrix<double, Eigen::RowMajor> CMJ;
      cnoidbodyutils::calcCMJacobian(robot,CMJ);

      cnoid::VectorXd wc(SCFR_M.rows());
      for(size_t i=0;i<wc.size();i++) wc[i] = 1.0;

      Cs.push_back(SCFR_M * CMJ.topRows<2>() * S);
      dls.push_back(SCFR_l);
      dus.push_back(SCFR_u);
      wcs.push_back(wc);
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

  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupPCLCollisionTask(std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                         cnoid::Body* robot,
                                                         std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                         std::shared_ptr<PCLCollisionDetector>& pCLCollisionDetector,
                                                         double tolerance,
                                                         double w,
                                                         double we){
    if(!this->pCLCollisionTask_) {
      this->pCLCollisionTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->pCLCollisionTask_;
      task->name() = "PCLCollisionTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->pCLCollisionTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 0;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> S(6+robot->numJoints(),cols);
    {
      for(size_t i=0;i<6;i++) S.insert(i,i) = 1.0;
      size_t idx = 6;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()){
          S.insert(6+i,idx) = 1.0;
          idx++;
        }
      }
    }

    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
    std::vector<cnoid::VectorXd> bs;
    std::vector<cnoid::VectorXd> was;
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorXd> dls;
    std::vector<cnoid::VectorXd> dus;
    std::vector<cnoid::VectorXd> wcs;

    // PCL Collision
    // 各行mのオーダ
    if(pCLCollisionDetector->solve()){
      cnoidbodyutils::Collision collision(robot);
      collision.tolerance() = tolerance;
      for(size_t i=0;i<pCLCollisionDetector->collisionLinkPairs().size();i++){
        collision.collisionLinkPair() = *(pCLCollisionDetector->collisionLinkPairs()[i]);

        Eigen::SparseMatrix<double,Eigen::RowMajor> A;
        cnoid::VectorX b;
        cnoid::VectorX wa;
        Eigen::SparseMatrix<double,Eigen::RowMajor> C;
        cnoid::VectorX dl;
        cnoid::VectorX du;
        cnoid::VectorX wc;
        collision.getCollisionConstraint(A,b,wa,C,dl,du,wc);
        As.push_back(A*S);
        bs.push_back(b);
        was.push_back(wa);
        Cs.push_back(C*S);
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

  // メンバ変数はdebug_print_とTask_しか使わない
  bool ContactBreakAbilityChecker::setupSCFRBreakTask(std::shared_ptr<prioritized_qp::Task>& taskHelper, //返り値
                                                      std::shared_ptr<prioritized_qp::Task>& task, //返り値
                                                      cnoid::Body* robot,
                                                      std::vector<std::shared_ptr<cnoidbodyutils::JointInfo> >& jointInfos,
                                                      const Eigen::SparseMatrix<double,Eigen::RowMajor>& SCFR_M,
                                                      const cnoid::VectorXd& SCFR_l,
                                                      const cnoid::VectorX& SCFR_u,
                                                      double w){
    if(!this->sCFRBreakTaskHelper_) {
      this->sCFRBreakTaskHelper_ = std::make_shared<prioritized_qp::Task>();
      task = this->sCFRBreakTaskHelper_;
      task->name() = "SCFRBreakTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = false; //解かない
    }else{
      task = this->sCFRBreakTaskHelper_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    if(!this->sCFRBreakTask_) {
      this->sCFRBreakTask_ = std::make_shared<prioritized_qp::Task>();
      task = this->sCFRBreakTask_;
      task->name() = "SCFRBreakTask";
      task->solver().settings()->resetDefaultSettings();
      task->solver().settings()->setVerbosity(debug_print_);
      task->solver().settings()->setWarmStart(true);
      task->solver().settings()->setMaxIteration(4000);
      task->solver().settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
      task->solver().settings()->setScaledTerimination(true);// avoid too severe termination check
      task->toSolve() = true;
    }else{
      task = this->sCFRBreakTask_;
      if(task->solver().settings()->getSettings()->verbose != debug_print_){
        task->solver().settings()->setVerbosity(debug_print_);
        task->solver().clearSolver();
      }
    }

    size_t cols = 0;
    for(size_t i=0;i<jointInfos.size();i++){
      if(jointInfos[i]->controllable()) cols++;
    }
    Eigen::SparseMatrix<double, Eigen::RowMajor> S(6+robot->numJoints(),cols);
    {
      for(size_t i=0;i<6;i++) S.insert(i,i) = 1.0;
      size_t idx = 6;
      for(size_t i=0;i<jointInfos.size();i++){
        if(jointInfos[i]->controllable()){
          S.insert(6+i,idx) = 1.0;
          idx++;
        }
      }
    }

    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > As;
    std::vector<cnoid::VectorXd> bs;
    std::vector<cnoid::VectorXd> was;
    std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> > Cs;
    std::vector<cnoid::VectorXd> dls;
    std::vector<cnoid::VectorXd> dus;
    std::vector<cnoid::VectorXd> wcs;

    {
      // 重心変位
      // 各行はm,radのオーダ
      // calc CM jacobian
      Eigen::SparseMatrix<double, Eigen::RowMajor> CMJ;
      cnoidbodyutils::calcCMJacobian(robot,CMJ);

      cnoid::VectorXd wc(SCFR_M.rows());
      for(size_t i=0;i<wc.size();i++) wc[i] = 1.0;

      Cs.push_back(SCFR_M * CMJ.topRows<2>() * S);
      dls.push_back(SCFR_l);
      dus.push_back(SCFR_u);
      wcs.push_back(wc);
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

    task->w().resize(cols);
    for(size_t i=0;i<task->w().size();i++)task->w()[i] = w;

    return true;
  }

};
