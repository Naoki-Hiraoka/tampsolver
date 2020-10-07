#ifndef CLPEIGEN_H
#define CLPEIGEN_H

#include <coin/ClpSimplex.hpp>
#include <Eigen/Eigen>
#include <Eigen/Sparse>

namespace clpeigen{
  class solver{
  public:
    solver();
    /*
      objective:  mazimize ox
      subject to: lbA <= Ax <= ubA
                  lb <= x <= ub
     */
    bool initialize(const Eigen::VectorXd& o,
                    const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                    const Eigen::VectorXd& lbA,
                    const Eigen::VectorXd& ubA,
                    const Eigen::VectorXd& lb,
                    const Eigen::VectorXd& ub,
                    int debuglevel=0);

    bool solve();

    bool getSolution(Eigen::VectorXd& solution);

    bool updateObjective(const Eigen::VectorXd& o);

    bool is_initialized() {return initialized;}
  private:
    ClpSimplex model;
    bool initial_solve;
    bool initialized;
  };
}

#endif
