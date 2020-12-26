#include "OsqpEigenSolver.h"
#include <cnoid/TimeMeasure>

namespace IK{
  OsqpEigenSolver::OsqpEigenSolver():
    debuglevel(0)
  {
  }

  bool OsqpEigenSolver::init(const Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                             const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                             const Eigen::VectorXd& _gradient,
                             const Eigen::VectorXd& _upperBound,
                             const Eigen::VectorXd& _lowerBound,
                             int _debuglevel){
    this->debuglevel = _debuglevel;

    solver.settings()->resetDefaultSettings();

    solver.settings()->setVerbosity(this->debuglevel);
    solver.settings()->setWarmStart(true);

    //solver.settings()->setRho(1e-6);
    //solver.settings()->setAlpha(0.1);
    solver.settings()->setMaxIteraction(4000);
    solver.settings()->setRho(1e-6);
    solver.settings()->setAbsoluteTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
    solver.settings()->setRelativeTolerance(1e-4);// 1e-5の方がいいかも．1e-4の方がやや速いが，やや不正確
    //solver.settings()->setAbsoluteTolerance(1e-5);// improve accuracy
    //solver.settings()->setRelativeTolerance(1e-5);// improve accuracy
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
                               const Eigen::VectorXd& lowerBound,
                               int _debuglevel){
    this->debuglevel = _debuglevel;
    solver.settings()->setVerbosity(this->debuglevel);

    solver.updateHessianMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(H)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
    solver.updateGradient(gradient);
    solver.updateLinearConstraintsMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(A)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
    solver.updateBounds(lowerBound,upperBound);//upperとlower同時にupdateしないと，一時的にupperがlowerを下回ってエラーになる
  }


  bool OsqpEigenSolver::solve() {
    cnoid::TimeMeasure timer;
    if(this->debuglevel>0){
      timer.begin();
    }

    bool solved = solver.solve();

    if(this->debuglevel>0){
      double time = timer.measure();
      std::cerr << " SQPEigen solve time: " << time
                << std::endl;
    }

    if(solved){
      return true;
    }else{
      if(this->debuglevel>0){
        timer.begin();
      }
      solver.clearSolverVariables();
      bool solved2 = solver.solve();
      if(this->debuglevel>0){
        double time = timer.measure();
        std::cerr << " SQPEigen solve2 time: " << time
                  << std::endl;
      }
      return solved2;
    }
  }

  Eigen::VectorXd  OsqpEigenSolver::getSolution(){
    return solver.getSolution();
  }

  bool OsqpEigenSolver::is_initialized(){
    return solver.isInitialized();
  }

  void OsqpEigenSolver::setTolerance(double tolerance){
    solver.settings()->setAbsoluteTolerance(tolerance);
    solver.settings()->setRelativeTolerance(tolerance);
  }
}
