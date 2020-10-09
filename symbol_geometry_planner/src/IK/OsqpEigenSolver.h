#ifndef OSQPEIGENSOLVER_H
#define OSQPEIGENSOLVER_H

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

namespace IK{
  class OsqpEigenSolver
  {
  public:
    OsqpEigenSolver();

    bool init(const Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
              const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
              const Eigen::VectorXd& gradient,
              const Eigen::VectorXd& upperBound,
              const Eigen::VectorXd& lowerBound,
              int debuglevel=0);

    bool update(const Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                const Eigen::VectorXd& gradient,
                const Eigen::VectorXd& upperBound,
                const Eigen::VectorXd& lowerBound,
                int debuglevel=0);
    bool solve();
    Eigen::VectorXd getSolution();
    bool is_initialized();

    void setTolerance(double tolerance);
  private:
    OsqpEigen::Solver solver;
    Eigen::VectorXd gradient;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lowerBound;

    int debuglevel;

  };
}

#endif
