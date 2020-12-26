#include "IKsolver.h"
#include <cnoid/TimeMeasure>

namespace IK{
  IKsolver::IKsolver(const std::vector<cnoid::Body*>& _variables,
                     const std::vector<std::shared_ptr<IKConstraint> >& _tasks,
                     const std::vector<std::shared_ptr<IKConstraint> >& _constraints):
    debuglevel(0),
    variables(_variables),
    tasks(_tasks),
    constraints(_constraints),
    regular(1e-6),
    regular_rel(5),
    regular_max(0.1),
    maxvel(0.1),
    eps(1e-3)
  {

  }

  // 解けたらtrue
  bool IKsolver::solve_one_loop(){
    cnoid::TimeMeasure timer;

    // calc qp matrix
    if(this->debuglevel>0){
      timer.begin();
    }

    double sum_error = this->calc_qp_matrix();

    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " IKsolver calc_qp_matrix time: " << time
                << std::endl;
    }

    // return if ik solved
    if(sum_error<this->eps*this->eps){
      bool satisfied = true;
      for(size_t i=0;i<this->upperBound.rows();i++){
        if(this->upperBound[i]<-this->eps||this->lowerBound[i]>this->eps){
          satisfied = false;
          break;
        }
      }
      if(satisfied) return true;
    }

    if(debuglevel>1){
      std::cerr << "H" << std::endl;
      std::cerr << this->H << std::endl;
      std::cerr << "gradient" << std::endl;
      std::cerr << this->gradient << std::endl;
      std::cerr << "A" << std::endl;
      std::cerr << this->A << std::endl;
      std::cerr << "lowerBound" << std::endl;
      std::cerr << this->lowerBound << std::endl;
      std::cerr << "upperBound" << std::endl;
      std::cerr << this->upperBound << std::endl;
    }

    // update qpsolver
    if(this->debuglevel>0){
      timer.begin();
    }

    if(!osqpeigensolver.is_initialized()){
      osqpeigensolver.init(this->H,this->A,this->gradient,this->upperBound,this->lowerBound,this->debuglevel);
    }else{
      osqpeigensolver.update(this->H,this->A,this->gradient,this->upperBound,this->lowerBound,this->debuglevel);
    }

    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " IKsolver update qpsolver time: " << time
                << std::endl;
    }

    // solve qp
    if(this->debuglevel>0){
      timer.begin();
    }

    if(osqpeigensolver.solve()){

      if(this->debuglevel>0){
        double time = timer.measure();
        std::cerr << " IKsolver solve qp time: " << time
                  << std::endl;
      }

      // update system

      if(this->debuglevel>0){
        timer.begin();
      }

      Eigen::VectorXd solution = osqpeigensolver.getSolution();

      update_qp_variables(solution);

      if(this->debuglevel>0){
        double time = timer.measure();
        std::cerr << " IKsolver update system time: " << time
                  << std::endl;
      }

      if(debuglevel>1){
        std::cerr << "solution" << std::endl;
        std::cerr << solution << std::endl;
        std::cerr << "objective" << std::endl;
        std::cerr << 0.5 * solution.transpose() * this->H * solution + this->gradient.transpose() * solution << std::endl;
        std::cerr << "this->Ax" << std::endl;
        std::cerr << this->A * solution << std::endl;
      }
    }else{
      std::cerr << "[IKsolver] qpfail" << std::endl;
    }


    return false;
  }

  bool IKsolver::solve_optimization() {
    //TODO
    return true;
  }

  std::vector<cnoid::SgNodePtr> IKsolver::getDrawOnObjects(){
    std::vector<cnoid::SgNodePtr> objects;
    for(size_t i=0; i<tasks.size();i++){
      std::vector<cnoid::SgNodePtr> tmp_objects = tasks[i]->getDrawOnObjects();
      std::copy(tmp_objects.begin(), tmp_objects.end(), std::back_inserter(objects));
    }
    for(size_t i=0; i<constraints.size();i++){
      std::vector<cnoid::SgNodePtr> tmp_objects = constraints[i]->getDrawOnObjects();
      std::copy(tmp_objects.begin(), tmp_objects.end(), std::back_inserter(objects));
    }
    return objects;
  }

  double IKsolver::calc_qp_matrix(){
    cnoid::TimeMeasure timer;

    int num_variables = 0;
    for(size_t i=0;i<this->variables.size();i++){
      num_variables += 6 + this->variables[i]->numJoints();
    }

    // H gradient
    if(this->debuglevel>0){
      timer.begin();
    }
    this->H = Eigen::SparseMatrix<double,Eigen::RowMajor>(num_variables,num_variables);
    this->gradient = Eigen::VectorXd::Zero(num_variables);
    double sum_error = 0;
    for(size_t i=0;i<this->tasks.size();i++){
      Eigen::VectorXd error = this->tasks[i]->calc_error();
      Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian = this->tasks[i]->calc_jacobian(this->variables);

      this->H += Eigen::SparseMatrix<double,Eigen::RowMajor>(jacobian.transpose() * jacobian);
      this->gradient += jacobian.transpose() * error;
      sum_error += error.squaredNorm();
    }
    //regular term
    double weight = std::min(this->regular + this->regular_rel * sum_error, this->regular_max);
    for(size_t i=0;i<num_variables;i++){
      this->H.coeffRef(i,i) += weight;
    }
    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " IKsolver calc H gradient time: " << time
                << std::endl;
    }

    //A upperBound lowerBound
    if(this->debuglevel>0){
      timer.begin();
    }
    int num_constraints = 0;
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > errors;errors.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobians;jacobians.reserve(this->constraints.size());
    std::vector<std::reference_wrapper <const Eigen::VectorXd> > minineqs;minineqs.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxineqs;maxineqs.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianineqs;jacobianineqs.reserve(this->constraints.size());

    cnoid::TimeMeasure timer2;

    for(size_t i=0;i<this->constraints.size();i++){
      if(this->debuglevel>0){
        timer2.begin();
      }
      errors.emplace_back(this->constraints[i]->calc_error());
      jacobians.emplace_back(this->constraints[i]->calc_jacobian(this->variables));

      jacobianineqs.emplace_back(this->constraints[i]->calc_jacobianineq(this->variables));
      minineqs.emplace_back(this->constraints[i]->calc_minineq());
      maxineqs.emplace_back(this->constraints[i]->calc_maxineq());

      num_constraints += errors[i].get().rows()+minineqs[i].get().rows();
      if(this->debuglevel>0){
        double time = timer2.measure();
        std::cerr << i <<" IKsolver calc  time: " << time
                  << std::endl;
      }
    }
    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " IKsolver calc A upperBound lower Bound time1: " << time
                << std::endl;
      timer.begin();
    }
    this->A.resize(num_constraints,num_variables);
    this->lowerBound.resize(num_constraints);
    this->upperBound.resize(num_constraints);
    int idx = 0;
    for(size_t i=0;i<this->constraints.size();i++){
      this->A.middleRows(idx,errors[i].get().rows()) = jacobians[i].get();
      this->lowerBound.segment(idx,errors[i].get().rows()) = - errors[i].get();
      this->upperBound.segment(idx,errors[i].get().rows()) = - errors[i].get();
      idx += errors[i].get().rows();

      this->A.middleRows(idx,minineqs[i].get().rows()) = jacobianineqs[i].get();
      this->lowerBound.segment(idx,minineqs[i].get().rows()) = minineqs[i].get();
      this->upperBound.segment(idx,minineqs[i].get().rows()) = maxineqs[i].get();
      idx += minineqs[i].get().rows();
    }
    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " IKsolver calc A upperBound lower Bound time2: " << time
                << std::endl;
    }


    return sum_error;
  }

  void IKsolver::update_qp_variables(const Eigen::VectorXd& solution){
    Eigen::VectorXd solution_filtered = solution;
    for(size_t i=0;i<solution_filtered.rows();i++){
      solution_filtered[i] = std::min(std::max(solution_filtered[i],-maxvel),maxvel);
    }

    int idx = 0;
    for(size_t i=0;i<this->variables.size();i++){
      variables[i]->rootLink()->p() += solution_filtered.segment<3>(idx);

      cnoid::Matrix3 dR = cnoid::Matrix3(cnoid::AngleAxis(solution_filtered.segment<3>(idx+3).norm(), solution_filtered.segment<3>(idx+3).normalized()));
      variables[i]->rootLink()->R() = dR * variables[i]->rootLink()->R();

      for(size_t j=0;j<variables[i]->numJoints();j++){
        variables[i]->joint(j)->q() += solution_filtered[idx+6+j];
      }

      this->variables[i]->calcForwardKinematics();
      this->variables[i]->calcCenterOfMass();

      idx += 6 + this->variables[i]->numJoints();
    }

    return;
  }
}
