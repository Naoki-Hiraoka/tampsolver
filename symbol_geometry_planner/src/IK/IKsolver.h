#ifndef IKSOLVER_H
#define IKSOLVER_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include "Constraints/IKConstraint.h"
#include "OsqpEigenSolver.h"

namespace IK{
  class IKsolver
  {
  public:
    IKsolver(const std::vector<cnoid::Body*>& variables,
             const std::vector<std::shared_ptr<IKConstraint> >& tasks,
             const std::vector<std::shared_ptr<IKConstraint> >& constraints);

    // 解けたらtrue
    virtual bool solve_one_loop();

    virtual bool solve_optimization();

    virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    void set_regular(double _regular){ regular = _regular;}
    void set_regular_rel(double _regular_rel){ regular_rel = _regular_rel;}
    void set_regular_max(double _regular_max){ regular_max = _regular_max;}
    void set_debug_level(int _debuglevel){ debuglevel = _debuglevel;}
    void set_maxvel(double _maxvel){ maxvel = maxvel;}
    void set_eps(double _maxvel){ maxvel = eps;}
  protected:
    // errorの和を返す
    virtual double calc_qp_matrix();
    virtual void update_qp_variables(const Eigen::VectorXd& solution);

    int debuglevel;

    const std::vector<cnoid::Body*> variables;
    const std::vector<std::shared_ptr<IKConstraint> > tasks;
    const std::vector<std::shared_ptr<IKConstraint> > constraints;

    OsqpEigenSolver osqpeigensolver;
    double regular;
    double regular_rel;
    double regular_max;
    double maxvel;
    double eps;

    Eigen::SparseMatrix<double,Eigen::RowMajor> H;
    Eigen::SparseMatrix<double,Eigen::RowMajor> A;
    Eigen::VectorXd gradient;
    Eigen::VectorXd upperBound;
    Eigen::VectorXd lowerBound;
  };
}

#endif
