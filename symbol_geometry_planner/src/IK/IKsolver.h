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

    // コストが増加したり，constraintsのエラーが閾値以上だとfalse
    virtual bool solve_one_loop();

    virtual bool solve_optimization();

    virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects();

    void set_regular(double _regular){ regular = _regular;}
    void set_regular_rel(double _regular_rel){ regular_rel = _regular_rel;}
    void set_regular_max(double _regular_max){ regular_max = _regular_max;}
    void set_debug_level(int _debuglevel){ debuglevel = _debuglevel;}
    void set_maxvel(double _maxvel){ maxvel = maxvel;}
  protected:
    virtual void calc_qp_matrix(Eigen::SparseMatrix<double,Eigen::RowMajor>& H,
                                Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                                Eigen::VectorXd& gradient,
                                Eigen::VectorXd& upperBound,
                                Eigen::VectorXd& lowerBound);
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
  };
}

#endif
