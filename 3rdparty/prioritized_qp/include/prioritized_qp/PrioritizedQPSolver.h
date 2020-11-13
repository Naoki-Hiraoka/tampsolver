#ifndef PRIORITIZEDQPSOLVER_H
#define PRIORITIZEDQPSOLVER_H

#include <OsqpEigen/OsqpEigen.h>
#include <Eigen/Dense>

namespace prioritized_qp{
  class Task
  {
    /*
      Ax = b
      dl <= Cx <= du
     */
  public:
    Eigen::SparseMatrix<double,Eigen::RowMajor> A() const { return A_; }
    Eigen::SparseMatrix<double,Eigen::RowMajor>& A() { return A_; }

    Eigen::VectorXd b() const { return b_; }
    Eigen::VectorXd& b() { return b_; }

    Eigen::VectorXd wa() const { return wa_; }
    Eigen::VectorXd& wa() { return wa_; }

    Eigen::SparseMatrix<double,Eigen::RowMajor> C() const { return C_; }
    Eigen::SparseMatrix<double,Eigen::RowMajor>& C() { return C_; }

    Eigen::VectorXd dl() const { return dl_; }
    Eigen::VectorXd& dl() { return dl_; }
    Eigen::VectorXd du() const { return du_; }
    Eigen::VectorXd& du() { return du_; }

    Eigen::VectorXd wc() const { return wc_; }
    Eigen::VectorXd& wc() { return wc_; }

    Eigen::VectorXd w() const { return w_; }
    Eigen::VectorXd& w() { return w_; }

    // settingsの設定はユーザーが行うこと
    OsqpEigen::Solver& solver() { return solver_; }

    bool toSolve() const {return toSolve_; }
    bool& toSolve() {return toSolve_; }

  private:
    Eigen::SparseMatrix<double,Eigen::RowMajor> A_;
    Eigen::VectorXd b_;
    Eigen::VectorXd wa_;
    Eigen::SparseMatrix<double,Eigen::RowMajor> C_;
    Eigen::VectorXd dl_;
    Eigen::VectorXd du_;
    Eigen::VectorXd wc_;

    Eigen::VectorXd w_;

    OsqpEigen::Solver solver_;
    bool toSolve_;
  };

  bool solve(std::vector< std::shared_ptr<Task> >& tasks, Eigen::VectorXd& result, int debuglevel=0);

}

#endif
