#include <prioritized_qp/PrioritizedQPSolver.h>

namespace prioritized_qp{
  bool solve(std::vector< std::shared_ptr<Task> >& tasks, Eigen::VectorXd& result, int debuglevel){
    if(tasks.size()==0) return false;

    // 次元をチェック
    int dim = -1;
    for(size_t i=0;i<tasks.size();i++){
      if(dim<0) dim = tasks[i]->A().cols();
      if(dim != tasks[i]->A().cols()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(dim != tasks[i]->C().cols()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->toSolve() && (dim != tasks[i]->w().cols() || dim != tasks[i]->w().rows() )){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }

      if(tasks[i]->A().rows() != tasks[i]->b().size()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->toSolve() && tasks[i]->A().rows() != tasks[i]->wa().size()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->C().rows() != tasks[i]->dl().size() || tasks[i]->C().rows() != tasks[i]->du().size()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->toSolve() && tasks[i]->C().rows() != tasks[i]->wc().size()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
    }

    Eigen::VectorXd solution;
    Eigen::SparseMatrix<double,Eigen::RowMajor> As(0,dim);
    Eigen::VectorXd lBs(0);
    Eigen::VectorXd uBs(0);

    for(size_t i=0;i<tasks.size();i++){
      As.resize(As.rows()+tasks[i]->A().rows(),dim);
      As.bottomRows(tasks[i]->A().rows()) = tasks[i]->A();
      lBs.resize(lBs.size()+tasks[i]->b().size());
      uBs.resize(uBs.size()+tasks[i]->b().size());
      lBs.tail(tasks[i]->b().size()) = tasks[i]->b();
      uBs.tail(tasks[i]->b().size()) = tasks[i]->b();
      As.resize(As.rows()+tasks[i]->C().rows(),dim);
      As.bottomRows(tasks[i]->C().rows()) = tasks[i]->C();
      lBs.resize(lBs.size()+tasks[i]->dl().size());
      uBs.resize(uBs.size()+tasks[i]->du().size());
      lBs.tail(tasks[i]->dl().size()) = tasks[i]->dl();
      uBs.tail(tasks[i]->du().size()) = tasks[i]->du();

      if(tasks[i]->toSolve()){

        int num = dim + tasks[i]->A().rows() + tasks[i]->C().rows();

        /*
          min 0.5 xHx + gx
          s.t lB <= Ax <= cB
         */
        Eigen::SparseMatrix<double,Eigen::ColMajor> H(num,num);
        Eigen::SparseMatrix<double,Eigen::ColMajor> A(As.rows(),num);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num);
        Eigen::VectorXd upperBound = uBs;
        Eigen::VectorXd lowerBound = lBs;

        for(size_t i=0;i<dim;i++) H.insert(i,i) = tasks[i]->w()(i);
        for(size_t i=0;i<tasks[i]->wa().size();i++) H.insert(dim+i,dim+i) = tasks[i]->wa()(i);
        for(size_t i=0;i<tasks[i]->wc().size();i++) H.insert(dim+tasks[i]->wa().size()+i,dim+tasks[i]->wa().size()+i) = tasks[i]->wc()(i);

        A.leftCols(As.cols()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(As);
        for(size_t i=0;i<tasks[i]->A().rows() + tasks[i]->C().rows();i++){
          As.insert(As.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + i, dim + i) = 1;
        }

        if(!tasks[i]->solver().isInitialized() ||
           tasks[i]->solver().workspace()->data->n != H.rows() ||
           tasks[i]->solver().workspace()->data->m != A.rows()
           ){
          if(tasks[i]->solver().isInitialized()) tasks[i]->solver().clearSolver();

          tasks[i]->solver().data()->setNumberOfVariables(H.cols());
          tasks[i]->solver().data()->setNumberOfConstraints(A.rows());
          tasks[i]->solver().data()->setHessianMatrix(H);
          tasks[i]->solver().data()->setGradient(gradient);
          tasks[i]->solver().data()->setLinearConstraintsMatrix(A);
          tasks[i]->solver().data()->setLowerBound(lowerBound);
          tasks[i]->solver().data()->setUpperBound(upperBound);
          tasks[i]->solver().initSolver();
        }else{
          tasks[i]->solver().updateHessianMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(H)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
          tasks[i]->solver().updateGradient(gradient);
          tasks[i]->solver().updateLinearConstraintsMatrix(Eigen::SparseMatrix<double,Eigen::ColMajor>(A)); // OsqpEigenの実装の都合上，ColMajorでないとサイレントにバグが起こる
          tasks[i]->solver().updateBounds(lowerBound,upperBound);//upperとlower同時にupdateしないと，一時的にupperがlowerを下回ってエラーになる
        }

        bool solved = tasks[i]->solver().solve();
        if(!solved) {
          tasks[i]->solver().clearSolverVariables();
          solved = tasks[i]->solver().solve();
        }
        if(!solved) return false;

        solution = tasks[i]->solver().getSolution();
        Eigen::VectorXd this_b = tasks[i]->A() * solution;
        Eigen::VectorXd this_d = tasks[i]->C() * solution;
        for(size_t i=0;i<tasks[i]->A().rows();i++){
          if(this_b(i)>tasks[i]->b()(i)) uBs(uBs.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + i) = this_b(i);
          if(this_b(i)<tasks[i]->b()(i)) lBs(lBs.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + i) = this_b(i);
        }
        for(size_t i=0;i<tasks[i]->C().rows();i++){
          if(this_d(i)>tasks[i]->du()(i)) uBs(uBs.rows() - tasks[i]->C().rows() + i) = this_d(i);
          if(this_b(i)<tasks[i]->dl()(i)) lBs(lBs.rows() - tasks[i]->C().rows() + i) = this_d(i);
        }
      }
    }

    result = solution;
    return true;
  }

};
