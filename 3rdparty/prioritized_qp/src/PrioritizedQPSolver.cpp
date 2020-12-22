#include <prioritized_qp/PrioritizedQPSolver.h>

namespace prioritized_qp{
  bool solve(std::vector< std::shared_ptr<Task> >& tasks, Eigen::VectorXd& result, int debuglevel){
    if(tasks.size()==0) {
      std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] no task is provided" << std::endl;
      return false;
    }

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
      if(tasks[i]->toSolve() && (dim != tasks[i]->w().size())){
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
      if(tasks[i]->A_ext().cols() != tasks[i]->C_ext().cols()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->A_ext().cols() != 0 && tasks[i]->A_ext().rows() != tasks[i]->A().rows()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->C_ext().cols() != 0 && tasks[i]->C_ext().rows() != tasks[i]->C().rows()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
      if(tasks[i]->toSolve() && tasks[i]->A_ext().cols() != tasks[i]->w_ext().size()){
        std::cerr << "[prioritized_qp::PriroritizedQPSolver::solve] dimension mismatch" << std::endl;
        return false;
      }
    }

    if (dim==0){
      result = Eigen::VectorXd(0);
      return true;
    }

    Eigen::VectorXd solution = Eigen::VectorXd::Zero(dim);
    Eigen::SparseMatrix<double,Eigen::RowMajor> As(0,dim);
    Eigen::VectorXd lBs(0);
    Eigen::VectorXd uBs(0);
    Eigen::VectorXd w_exts(0);

    for(size_t i=0;i<tasks.size();i++){
      Eigen::SparseMatrix<double,Eigen::RowMajor> taskA = tasks[i]->A();
      Eigen::SparseMatrix<double,Eigen::RowMajor> taskC = tasks[i]->C();
      if(tasks[i]->A_ext().cols() != 0){
        Eigen::SparseMatrix<double,Eigen::ColMajor> taskA_ColMajor(taskA.rows(), As.cols() + tasks[i]->A_ext().cols());
        taskA_ColMajor.leftCols(taskA.cols()) = taskA;
        taskA_ColMajor.rightCols(tasks[i]->A_ext().cols()) = tasks[i]->A_ext();
        taskA = taskA_ColMajor;
        Eigen::SparseMatrix<double,Eigen::ColMajor> taskC_ColMajor(taskC.rows(), As.cols() + tasks[i]->C_ext().cols());
        taskC_ColMajor.leftCols(taskC.cols()) = taskC;
        taskC_ColMajor.rightCols(tasks[i]->C_ext().cols()) = tasks[i]->C_ext();
        taskC = taskC_ColMajor;
        w_exts.resize(w_exts.size() + tasks[i]->w_ext().size());
        w_exts.tail(tasks[i]->w_ext().size()) = tasks[i]->w_ext();
      }

      As.conservativeResize(As.rows()+taskA.rows(),taskA.cols());
      As.bottomRows(taskA.rows()) = taskA;
      lBs.conservativeResize(lBs.size()+tasks[i]->b().size());
      uBs.conservativeResize(uBs.size()+tasks[i]->b().size());
      lBs.tail(tasks[i]->b().size()) = tasks[i]->b();
      uBs.tail(tasks[i]->b().size()) = tasks[i]->b();
      As.conservativeResize(As.rows()+taskC.rows(),taskC.cols());
      As.bottomRows(taskC.rows()) = taskC;
      lBs.conservativeResize(lBs.size()+tasks[i]->dl().size());
      uBs.conservativeResize(uBs.size()+tasks[i]->du().size());
      lBs.tail(tasks[i]->dl().size()) = tasks[i]->dl();
      uBs.tail(tasks[i]->du().size()) = tasks[i]->du();

      if(tasks[i]->toSolve()){
        if(tasks[i]->A().rows()==0 && tasks[i]->C().rows()==0)continue;

        int num = As.cols() + tasks[i]->A().rows() + tasks[i]->C().rows();

        /*
          min 0.5 xHx + gx
          s.t lB <= Ax <= cB
         */
        Eigen::SparseMatrix<double,Eigen::ColMajor> H(num,num);
        Eigen::SparseMatrix<double,Eigen::ColMajor> A(As.rows(),num);
        Eigen::VectorXd gradient = Eigen::VectorXd::Zero(num);
        Eigen::VectorXd upperBound = uBs;
        Eigen::VectorXd lowerBound = lBs;

        for(size_t j=0;j<dim;j++) H.insert(j,j) = tasks[i]->w()(j);
        for(size_t j=0;j<w_exts.size();j++) H.insert(dim+j,dim+j) = w_exts(j);
        for(size_t j=0;j<tasks[i]->wa().size();j++) H.insert(As.cols()+j,As.cols()+j) = tasks[i]->wa()(j);
        for(size_t j=0;j<tasks[i]->wc().size();j++) H.insert(As.cols()+tasks[i]->wa().size()+j,As.cols()+tasks[i]->wa().size()+j) = tasks[i]->wc()(j);
        A.leftCols(As.cols()) = Eigen::SparseMatrix<double,Eigen::ColMajor>(As);
        for(size_t j=0;j<tasks[i]->A().rows() + tasks[i]->C().rows();j++){
          A.insert(As.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + j, As.cols() + j) = 1;
        }

        if(!tasks[i]->solver().isInitialized() ||
           tasks[i]->solver().workspace()->data->n != H.rows() ||
           tasks[i]->solver().workspace()->data->m != A.rows()
           ){
          tasks[i]->solver().data()->clearHessianMatrix();
          tasks[i]->solver().data()->clearLinearConstraintsMatrix();
          tasks[i]->solver().clearSolver();

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

        solution = tasks[i]->solver().getSolution().head(As.cols());
        Eigen::VectorXd this_b = taskA * solution;
        Eigen::VectorXd this_d = taskC * solution;
        for(size_t j=0;j<tasks[i]->A().rows();j++){
          if(this_b(j)>tasks[i]->b()(j)) uBs(uBs.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + j) = this_b(j);
          if(this_b(j)<tasks[i]->b()(j)) lBs(lBs.rows() - tasks[i]->A().rows() - tasks[i]->C().rows() + j) = this_b(j);
        }
        for(size_t j=0;j<tasks[i]->C().rows();j++){
          if(this_d(j)>tasks[i]->du()(j)) uBs(uBs.rows() - tasks[i]->C().rows() + j) = this_d(j);
          if(this_b(j)<tasks[i]->dl()(j)) lBs(lBs.rows() - tasks[i]->C().rows() + j) = this_d(j);
        }
      }
    }

    result = solution.head(dim);
    return true;
  }

};
