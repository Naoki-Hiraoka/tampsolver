#include "OsqpEigenSolver.h"

namespace IK{
  bool OsqpEigenSolver::init(const Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                             const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                             const Eigen::VectorXd& _gradient,
                             const Eigen::VectorXd& _upperBound,
                             const Eigen::VectorXd& _lowerBound){
    solver.settings()->resetDefaultSettings();

    solver.settings()->setVerbosity(false);
    solver.settings()->setWarmStart(true);

    //solver.settings()->setRho(1e-6);
    //solver.settings()->setAlpha(0.1);
    solver.settings()->setMaxIteraction(4000);
    solver.settings()->setRho(1e-6);
    solver.settings()->setAbsoluteTolerance(1e-5);// improve accuracy
    solver.settings()->setRelativeTolerance(1e-5);// improve accuracy
    //settings->eps_abs = 1e-05;
    //settings->eps_rel = 1e-05;
    solver.settings()->setScaledTerimination(true);// avoid too severe termination check
    solver.settings()->setPolish(true);// improve accuracy. but cause oscillatory solution when convex error
    //settings->delta = 1e-4; // in polish, too small delta causes non-convex error, too large delta causes failure(unsuccessful)

    //The user has to guarantee that the lifetime of the object passed is the same of the OsqpEigen object
    gradient = _gradient;
    upperBound = _upperBound;
    lowerBound = _lowerBound;

    solver.data()->setNumberOfVariables(H.cols());
    solver.data()->setNumberOfConstraints(A.rows());
    solver.data()->setHessianMatrix(H);
    solver.data()->setGradient(gradient);
    solver.data()->setLinearConstraintsMatrix(A);
    solver.data()->setLowerBound(lowerBound);
    solver.data()->setUpperBound(upperBound);


    solver.initSolver();

    return true;
  }

  bool OsqpEigenSolver::update(const Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                               const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                               const Eigen::VectorXd& gradient,
                               const Eigen::VectorXd& upperBound,
                               const Eigen::VectorXd& lowerBound){
    solver.updateHessianMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(H)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
    solver.updateGradient(gradient);
    solver.updateLinearConstraintsMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(A)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
    solver.updateBounds(lowerBound,upperBound);//upperとlower同時にupdateしないと，一時的にupperがlowerを下回ってエラーになる
  }


  bool OsqpEigenSolver::solve() {
    if(solver.solve()){
      return true;
    }else{
      solver.clearSolverVariables();
      return solver.solve();
    }
  }

  Eigen::VectorXd  OsqpEigenSolver::getSolution(){
    return solver.getSolution();
  }

  bool OsqpEigenSolver::is_initialized(){
    return solver.isInitialized();
  }
}
