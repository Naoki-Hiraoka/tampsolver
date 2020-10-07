#ifndef CDDEIGEN_H
#define CDDEIGEN_H

#include <cdd/setoper.h>
#include <cdd/cdd.h>
#include <Eigen/Eigen>

namespace cddeigen{
  // gmpなしの方が速いが，計算が失敗しやすい

  /*
    INPUT:
      A_eq   x + b_eq    = 0
      A_ineq x + b_ineq >= 0
    OUTPUT:
      x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
  */

  bool HtoV (const Eigen::MatrixXd& A_eq,
             const Eigen::VectorXd& b_eq,
             const Eigen::MatrixXd& A_ineq,
             const Eigen::VectorXd& b_ineq,
             Eigen::MatrixXd& V,
             Eigen::MatrixXd& R_nonneg,
             Eigen::MatrixXd& R_free,
             bool verbose=false);

  bool HtoVgmp (const Eigen::MatrixXd& A_eq,
                const Eigen::VectorXd& b_eq,
                const Eigen::MatrixXd& A_ineq,
                const Eigen::VectorXd& b_ineq,
                Eigen::MatrixXd& V,
                Eigen::MatrixXd& R_nonneg,
                Eigen::MatrixXd& R_free,
                bool verbose=false,
                unsigned long int denom = 1000000);

  /*
    INPUT:
      x = V y + R_nonneg z + R_free w (sum y = 1, y >= 0, z >= 0)
    OUTPUT:
      A_eq   x + b_eq    = 0
      A_ineq x + b_ineq >= 0
  */

  bool VtoH (const Eigen::MatrixXd& V,
             const Eigen::MatrixXd& R_nonneg,
             const Eigen::MatrixXd& R_free,
             Eigen::MatrixXd& A_eq,
             Eigen::VectorXd& b_eq,
             Eigen::MatrixXd& A_ineq,
             Eigen::VectorXd& b_ineq,
             bool verbose=false);

  bool VtoHgmp (const Eigen::MatrixXd& V,
                const Eigen::MatrixXd& R_nonneg,
                const Eigen::MatrixXd& R_free,
                Eigen::MatrixXd& A_eq,
                Eigen::VectorXd& b_eq,
                Eigen::MatrixXd& A_ineq,
                Eigen::VectorXd& b_ineq,
                bool verbose=false,
                unsigned long int denom = 1000000);

};

#endif
