#ifndef OSQPEIGENSOLVER_H
#define OSQPEIGENSOLVER_H

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

class OsqpEigenSolver
{
public:
  bool init(const Eigen::SparseMatrix<double>& H,
            const Eigen::SparseMatrix<double>& A,
            const Eigen::VectorXd& gradient,
            const Eigen::VectorXd& upperBound,
            const Eigen::VectorXd& lowerBound);

  bool update(const Eigen::SparseMatrix<double>& H,
              const Eigen::SparseMatrix<double>& A,
              const Eigen::VectorXd& gradient,
              const Eigen::VectorXd& upperBound,
              const Eigen::VectorXd& lowerBound);
  bool solve();
  Eigen::VectorXd getSolution();
  bool is_initialized();
private:
  OsqpEigen::Solver solver;
  Eigen::VectorXd gradient;
  Eigen::VectorXd upperBound;
  Eigen::VectorXd lowerBound;
};

#endif
