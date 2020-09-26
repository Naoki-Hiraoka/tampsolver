#include "IKsolver.h"

namespace IK{
  IKsolver::IKsolver(const std::vector<cnoid::BodyItemPtr>& _variables,
                     const std::vector<std::shared_ptr<IKConstraint> >& _tasks,
                     const std::vector<std::shared_ptr<IKConstraint> >& _constraints):
    debuglevel(0),
    variables(_variables),
    tasks(_tasks),
    constraints(_constraints),
    regular(1e-6),
    regular_rel(5),
    regular_max(0.1)
  {

  }

  // コストが増加したり，constraintsのエラーが閾値以上だとfalse TODO
  bool IKsolver::solve_one_loop(){
    Eigen::SparseMatrix<double,Eigen::RowMajor> H;
    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd gradient;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lowerBound;

    this->calc_qp_matrix(H,A,gradient,upperBound,lowerBound);
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

    if(!osqpeigensolver.is_initialized()){
      osqpeigensolver.init(H,A,gradient,upperBound,lowerBound);
    }else{
      osqpeigensolver.update(H,A,gradient,upperBound,lowerBound);
    }

    osqpeigensolver.solve();

    Eigen::VectorXd solution = osqpeigensolver.getSolution();

    update_qp_variables(solution);

    if(debuglevel>1){
      std::cerr << "solution" << std::endl;
      std::cerr << solution << std::endl;
      std::cerr << "objective" << std::endl;
      std::cerr << 0.5 * solution.transpose() * H * solution + gradient.transpose() * solution << std::endl;
      std::cerr << "Ax" << std::endl;
      std::cerr << A * solution << std::endl;
    }


    return true;
  }

  bool IKsolver::solve_optimization() {
    //TODO
    return true;
  }

  void IKsolver::calc_qp_matrix(Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                                Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                                Eigen::VectorXd& gradient,
                                Eigen::VectorXd& upperBound,
                                Eigen::VectorXd& lowerBound){
    int num_variables = 0;
    for(size_t i=0;i<this->variables.size();i++){
      num_variables += 6 + this->variables[i]->body()->numJoints();
    }

    // H gradient
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


    //A upperBound lowerBound
    std::vector<Eigen::VectorXd> errors(this->constraints.size());
    std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > jacobians(this->constraints.size());
    int num_constraints = 0;
    for(size_t i=0;i<this->constraints.size();i++){
      errors[i] = this->constraints[i]->calc_error();
      jacobians[i] = this->constraints[i]->calc_jacobian(this->variables);
      num_constraints += errors[i].rows();
    }
    A = Eigen::SparseMatrix<double,Eigen::RowMajor>(num_constraints,num_variables);
    lowerBound = Eigen::VectorXd(num_constraints);
    upperBound = Eigen::VectorXd(num_constraints);
    int idx = 0;
    for(size_t i=0;i<this->constraints.size();i++){
      A.middleRows(idx,errors[i].rows()) = jacobians[i];
      lowerBound.segment(idx,errors[i].rows()) = - errors[i];
      upperBound.segment(idx,errors[i].rows()) = - errors[i];
      idx += errors[i].rows();
    }

    return;
  }

  void IKsolver::update_qp_variables(const Eigen::VectorXd& solution){
    int idx = 0;
    for(size_t i=0;i<this->variables.size();i++){
      variables[i]->body()->rootLink()->p() += solution.segment<3>(idx);

      cnoid::Matrix3 dR = cnoid::Matrix3(cnoid::AngleAxis(solution.segment<3>(idx+3).norm(), solution.segment<3>(idx+3).normalized()));
      variables[i]->body()->rootLink()->R() = dR * variables[i]->body()->rootLink()->R();

      for(size_t j=0;j<variables[i]->body()->numJoints();j++){
        variables[i]->body()->joint(j)->q() += solution[idx+6+j];
      }

      this->variables[i]->body()->calcForwardKinematics();

      idx += 6 + this->variables[i]->body()->numJoints();
    }

    return;
  }
}
