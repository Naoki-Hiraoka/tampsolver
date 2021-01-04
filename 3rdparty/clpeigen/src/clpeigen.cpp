#include <clpeigen/clpeigen.h>

namespace clpeigen{
  solver::solver():
    initialized(false),
    initial_solve(true)
  {
  }

  bool solver::initialize(const Eigen::VectorXd& o,
                          const Eigen::SparseMatrix<double,Eigen::RowMajor>& A,
                          const Eigen::VectorXd& lbA,
                          const Eigen::VectorXd& ubA,
                          const Eigen::VectorXd& lb,
                          const Eigen::VectorXd& ub,
                          int debuglevel){
    int numberRows = A.rows();
    int numberColumns = A.cols();
    int numberElements = A.nonZeros();//0の数ではなく要素数を見ている

    // matrix data - row ordered
    std::vector<int> len(A.rows());
    for(size_t i=0;i<A.rows();i++){
      len[i] = A.row(i).nonZeros();
    }
    CoinPackedMatrix matrix(false,//true: ColMajor, false: RowMajor
                            numberColumns,//minor
                            numberRows,//major
                            numberElements,
                            A.valuePtr(),
                            A.innerIndexPtr(),
                            A.outerIndexPtr(),
                            len.data());

    // load problem
    this->model.loadProblem(matrix,
                            lb.data(),
                            ub.data(),
                            o.data(),
                            lbA.data(),
                            ubA.data());
    this->model.setOptimizationDirection(-1);//maximize
    this->model.setLogLevel(debuglevel);

    this->initialized = true;
    this->initial_solve = true;
    return true;
  }

  bool solver::solve(){
    // Solve
    if(this->initial_solve){
      int status = this->model.initialSolve();
      this->initial_solve = false;
      return status == 0;
    }else{
      // Solve - primal as primal feasible
      int status = this->model.primal(1);
      return status == 0;
    }
  }

  bool solver::getSolution(Eigen::VectorXd& solution){
    solution.resize(this->model.getNumCols());

    // Solution
    const double * s = this->model.primalColumnSolution();
    for (size_t i=0; i < this->model.getNumCols(); i++) solution[i] = s[i];

    return true;
  }

  bool solver::updateObjective(const Eigen::VectorXd& o){
    if(o.rows() != this->model.getNumCols()){
      std::cerr << "[clpeigen::solver::updateObjective] dimention mismatch" << std::endl;
      return false;
    }

    double * objective = model.objective();
    for(size_t i=0;i<this->model.getNumCols();i++) objective[i] = o[i];

    return true;
  }
}
