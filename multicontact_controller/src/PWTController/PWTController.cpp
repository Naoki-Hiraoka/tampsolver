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
  void ContactPointPWTC::calcDesiredPositionConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, cnoid::VectorX& b, cnoid::VectorX& wa, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, cnoid::VectorX& dl, cnoid::VectorXd& du, cnoid::VectorX& wc){
    return;
  }


  bool PWTController::calcPWTControl(std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints){

    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > Js;Js.reserve(contactPoints.size());
    for(size_t i=0;i<contactPoints.size();i++) Js.emplace_back(contactPoints[i]->calcJacobian());

    // calc PWT Jacobian
    Eigen::SparseMatrix<double,Eigen::RowMajor> Dqa;
    Eigen::SparseMatrix<double,Eigen::RowMajor> Dwa;
    Eigen::SparseMatrix<double,Eigen::RowMajor> Dtaua;
    if(!this->calcPWTJacobian(Dqa, Dwa, Dtaua, contactPoints, Js)){
      std::cerr << "calcPWTJacobian failed" << std::endl;
      return false;
    }

    return true;
  }

  bool PWTController::calcPWTJacobian(Eigen::SparseMatrix<double,Eigen::RowMajor>& Dqa,//返り値
                                      Eigen::SparseMatrix<double,Eigen::RowMajor>& Dwa,//返り値
                                      Eigen::SparseMatrix<double,Eigen::RowMajor>& Dtaua,//返り値
                                      std::vector<std::shared_ptr<ContactPointPWTC> >& contactPoints,
                                      std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > >& Js){
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& Dg = torqueJacobianCalculator_.calcDg();
    const Eigen::SparseMatrix<double,Eigen::RowMajor>& DJw = torqueJacobianCalculator_.calcDJw(contactPoints); // F = 0 のcontactpointをここで入れるのは計算コストの無駄かもしれない

    for(size_t i=0;i<robot_->numJoints();i++){
      Ka_.coeffRef(6+i,6+i) = jointInfos_[i]->pgain();
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> Jbal;
    {
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Jbals;
      size_t num_rows=0;
      for(size_t i=0;i<contactPoints.size();i++) {
        Jbals.push_back(contactPoints[i]->selectMatrixForKinematicsConstraint() * Js[i].get());
        num_rows+=Jbals[i].rows();
      }
      Jbal.resize(num_rows,6+robot_->numJoints());
      size_t idx = 0;
      for(size_t i=0;i<Jbals.size();i++){
        Jbal.middleRows(idx,Jbals[i].rows()) = Jbals[i];
        idx += Jbals[i].rows();
      }
    }

    Eigen::SparseMatrix<double,Eigen::RowMajor> X(6+robot_->numJoints()+Jbal.rows(),6+robot_->numJoints()+Jbal.rows());
    {
      Eigen::SparseMatrix<double,Eigen::RowMajor> Xleftupper = Dg - DJw + Ka_;
      Eigen::SparseMatrix<double,Eigen::ColMajor> Xupper(6+robot_->numJoints(),6+robot_->numJoints()+Jbal.rows());
      Xupper.leftCols(6+robot_->numJoints()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(Xleftupper);
      Xupper.rightCols(Jbal.rows()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(-Jbal.transpose());
      Eigen::SparseMatrix<double,Eigen::ColMajor> Xlower(Jbal.rows(),6+robot_->numJoints()+Jbal.rows());
      Xlower.leftCols(6+robot_->numJoints()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(Jbal.transpose());
      X.topRows(Xupper.rows()) = Eigen::SparseMatrix<double,Eigen::RowMajor>(Xupper);
      X.bottomRows(Xlower.rows()) = Eigen::SparseMatrix<double,Eigen::RowMajor>(Xlower);
    }
    cnoid::MatrixXd Xinv;
    cnoidbodyutils::calcPseudoInverse(X,Xinv,sv_ratio_);
    Eigen::SparseMatrix<double,Eigen::RowMajor> Yu(6+robot_->numJoints(),Xinv.cols());
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
      for(size_t i=0;i<robot_->numJoints();i++){
        if(jointInfos_[i]->controllable()) num_cols++;
      }
      Kc.resize(Xinv.cols(),num_cols);
      size_t idx = 0;
      for(size_t i=0;i<robot_->numJoints();i++){
        if(jointInfos_[i]->controllable()){
          Kc.coeffRef(6+i,idx) = jointInfos_[i]->pgain();
          idx++;
        }
      }
    }
    return true;
  }

};
