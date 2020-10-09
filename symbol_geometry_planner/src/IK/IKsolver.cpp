#include "IKsolver.h"
#include <sys/time.h>

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
    struct timeval s, e;

    // calc qp matrix
    Eigen::SparseMatrix<double,Eigen::RowMajor> H;
    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd gradient;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lowerBound;

    if(this->debuglevel>0){
      gettimeofday(&s, NULL);
    }

    double sum_error = this->calc_qp_matrix(H,A,gradient,upperBound,lowerBound);

    if(this->debuglevel>0){
      gettimeofday(&e, NULL);
      std::cerr << " IKsolver calc_qp_matrix time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                << std::endl;
    }

    // return if ik solved
    if(sum_error<this->eps*this->eps){
      bool satisfied = true;
      for(size_t i=0;i<upperBound.rows();i++){
        if(upperBound[i]<-this->eps||lowerBound[i]>this->eps){
          satisfied = false;
          break;
        }
      }
      if(satisfied) return true;
    }

    if(debuglevel>1){
      std::cerr << "H" << std::endl;
      std::cerr << H << std::endl;
      std::cerr << "gradient" << std::endl;
      std::cerr << gradient << std::endl;
      std::cerr << "A" << std::endl;
      std::cerr << A << std::endl;
      std::cerr << "lowerBound" << std::endl;
      std::cerr << lowerBound << std::endl;
      std::cerr << "upperBound" << std::endl;
      std::cerr << upperBound << std::endl;
    }

    // update qpsolver
    if(this->debuglevel>0){
      gettimeofday(&s, NULL);
    }

    if(!osqpeigensolver.is_initialized()){
      osqpeigensolver.init(H,A,gradient,upperBound,lowerBound,this->debuglevel);
    }else{
      osqpeigensolver.update(H,A,gradient,upperBound,lowerBound,this->debuglevel);
    }

    if(this->debuglevel>0){
      gettimeofday(&e, NULL);
      std::cerr << " IKsolver update qpsolver time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                << std::endl;
    }

    // solve qp
    if(this->debuglevel>0){
      gettimeofday(&s, NULL);
    }

    if(osqpeigensolver.solve()){

      if(this->debuglevel>0){
        gettimeofday(&e, NULL);
        std::cerr << " IKsolver solve qp time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                  << std::endl;
      }

      // update system

      if(this->debuglevel>0){
        gettimeofday(&s, NULL);
      }

      Eigen::VectorXd solution = osqpeigensolver.getSolution();

      update_qp_variables(solution);

      if(this->debuglevel>0){
        gettimeofday(&e, NULL);
        std::cerr << " IKsolver update system time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                  << std::endl;
      }

      if(debuglevel>1){
        std::cerr << "solution" << std::endl;
        std::cerr << solution << std::endl;
        std::cerr << "objective" << std::endl;
        std::cerr << 0.5 * solution.transpose() * H * solution + gradient.transpose() * solution << std::endl;
        std::cerr << "Ax" << std::endl;
        std::cerr << A * solution << std::endl;
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

  double IKsolver::calc_qp_matrix(Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                                Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                                Eigen::VectorXd& gradient,
                                Eigen::VectorXd& upperBound,
                                Eigen::VectorXd& lowerBound){
    struct timeval s, e;

    int num_variables = 0;
    for(size_t i=0;i<this->variables.size();i++){
      num_variables += 6 + this->variables[i]->numJoints();
    }

    // H gradient
    if(this->debuglevel>0){
      gettimeofday(&s, NULL);
    }
    H = Eigen::SparseMatrix<double,Eigen::RowMajor>(num_variables,num_variables);
    gradient = Eigen::VectorXd::Zero(num_variables);
    double sum_error = 0;
    for(size_t i=0;i<this->tasks.size();i++){
      Eigen::VectorXd error = this->tasks[i]->calc_error();
      Eigen::SparseMatrix<double,Eigen::RowMajor> jacobian = this->tasks[i]->calc_jacobian(this->variables);

      H += Eigen::SparseMatrix<double,Eigen::RowMajor>(jacobian.transpose() * jacobian);
      gradient += jacobian.transpose() * error;
      sum_error += error.squaredNorm();
    }
    //regular term
    double weight = std::min(this->regular + this->regular_rel * sum_error, this->regular_max);
    for(size_t i=0;i<num_variables;i++){
      H.coeffRef(i,i) += weight;
    }
    if(this->debuglevel>0){
      gettimeofday(&e, NULL);
      std::cerr << " IKsolver calc H gradient time: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                << std::endl;
    }

    //A upperBound lowerBound
    if(this->debuglevel>0){
      gettimeofday(&s, NULL);
    }
    int num_constraints = 0;
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > errors;errors.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobians;jacobians.reserve(this->constraints.size());
    std::vector<std::reference_wrapper <const Eigen::VectorXd> > minineqs;minineqs.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::VectorXd> > maxineqs;maxineqs.reserve(this->constraints.size());
    std::vector<std::reference_wrapper<const Eigen::SparseMatrix<double,Eigen::RowMajor> > > jacobianineqs;jacobianineqs.reserve(this->constraints.size());

    struct timeval s2, e2;

    for(size_t i=0;i<this->constraints.size();i++){
      if(this->debuglevel>0){
        gettimeofday(&s2, NULL);
      }
      errors.emplace_back(this->constraints[i]->calc_error());
      jacobians.emplace_back(this->constraints[i]->calc_jacobian(this->variables));

      jacobianineqs.emplace_back(this->constraints[i]->calc_jacobianineq(this->variables));
      minineqs.emplace_back(this->constraints[i]->calc_minineq());
      maxineqs.emplace_back(this->constraints[i]->calc_maxineq());

      num_constraints += errors[i].get().rows()+minineqs[i].get().rows();
      if(this->debuglevel>0){
        gettimeofday(&e2, NULL);
        std::cerr << i <<" IKsolver calc  time: " << (e2.tv_sec - s2.tv_sec) + (e2.tv_usec - s2.tv_usec)*1.0E-6
                  << std::endl;
      }
    }
    if(this->debuglevel>0){
      gettimeofday(&e, NULL);
      std::cerr << " IKsolver calc A upperBound lower Bound time1: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
                << std::endl;
      gettimeofday(&s, NULL);
    }
    A = Eigen::SparseMatrix<double,Eigen::RowMajor>(num_constraints,num_variables);
    lowerBound = Eigen::VectorXd(num_constraints);
    upperBound = Eigen::VectorXd(num_constraints);
    int idx = 0;
    for(size_t i=0;i<this->constraints.size();i++){
      A.middleRows(idx,errors[i].get().rows()) = jacobians[i].get();
      lowerBound.segment(idx,errors[i].get().rows()) = - errors[i].get();
      upperBound.segment(idx,errors[i].get().rows()) = - errors[i].get();
      idx += errors[i].get().rows();

      A.middleRows(idx,minineqs[i].get().rows()) = jacobianineqs[i].get();
      lowerBound.segment(idx,minineqs[i].get().rows()) = minineqs[i].get();
      upperBound.segment(idx,minineqs[i].get().rows()) = maxineqs[i].get();
      idx += minineqs[i].get().rows();
    }
    if(this->debuglevel>0){
      gettimeofday(&e, NULL);
      std::cerr << " IKsolver calc A upperBound lower Bound time2: " << (e.tv_sec - s.tv_sec) + (e.tv_usec - s.tv_usec)*1.0E-6
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
