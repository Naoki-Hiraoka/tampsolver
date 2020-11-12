#include <cddeigen/cddeigen.h>
#include <iostream>

namespace cddeigen{
  static bool initialized=false;

  //ddをddfに変えるとgmpを使わなくなる

  bool HtoV (const Eigen::MatrixXd& A_eq,
             const Eigen::VectorXd& b_eq,
             const Eigen::MatrixXd& A_ineq,
             const Eigen::VectorXd& b_ineq,
             Eigen::MatrixXd& V,
             Eigen::MatrixXd& R_nonneg,
             Eigen::MatrixXd& R_free,
             bool verbose){
    if(!initialized) ddf_set_global_constants();

    if( (A_eq.rows() != b_eq.rows()) || (A_ineq.rows() != b_ineq.rows()) || (A_eq.cols() != A_ineq.cols())) {
      std::cerr << "[cddeigen::HtoV] dimention mismatch" << std::endl;
      return false;
    }

    // setup A
    ddf_MatrixPtr A = ddf_CreateMatrix(A_eq.rows() + A_ineq.rows(),A_eq.cols()+1);
    for (size_t i = 0; i < A_eq.rows(); i++){
      set_addelem(A->linset,i+1);

      ddf_set_d(A->matrix[i][0], b_eq[i]);
      for (size_t j = 0; j < A_eq.cols(); j++){
        ddf_set_d(A->matrix[i][j+1], A_eq(i,j));
      }
    }
    for (size_t i = 0; i < A_ineq.rows(); i++){
      ddf_set_d(A->matrix[A_eq.rows()+i][0], b_ineq[i]);
      for (size_t j = 0; j < A_ineq.cols(); j++){
        ddf_set_d(A->matrix[A_eq.rows()+i][j+1], A_ineq(i,j));
      }
    }
    A->representation=ddf_Inequality;

    if (verbose){
      std::cerr << "input H-representation:" << std::endl;
      ddf_WriteMatrix(stderr,A);
      std::cerr << std::endl;
    }

    ddf_rowindex newpos;
    ddf_rowset impl_linset,redset;
    ddf_ErrorType errf;
    ddf_MatrixCanonicalize(&A, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      std::cerr << "after canonicalize:" << std::endl;
      ddf_WriteMatrix(stderr,A);
      std::cerr << std::endl;
    }

    // compute the second (generator) representation
    ddf_ErrorType err;
    ddf_PolyhedraPtr poly=ddf_DDMatrix2Poly(A, &err);

    if (err==ddf_NoError){
      ddf_MatrixPtr G = ddf_CopyGenerators(poly);

      if (verbose){
        std::cerr <<"output V-representation:" <<std::endl;;
        ddf_WriteMatrix(stdout,G);
        std::cerr << std::endl;
      }

      // get matrix size
      int V_cols = 0;
      int R_nonneg_cols = 0;
      int R_free_cols = 0;
      for (size_t i = 0; i < G->rowsize; i++){
        if (ddf_get_d(G->matrix[i][0]) ==0 ){
          if (set_member(i+1,G->linset)) R_free_cols++;
          else R_nonneg_cols++;
        }else V_cols++;
      }

      // create return matrix
      V.resize(A_eq.cols(),V_cols);
      R_nonneg.resize(A_eq.cols(),R_nonneg_cols);
      R_free.resize(A_eq.cols(),R_free_cols);
      int V_idx = 0;
      int R_nonneg_idx = 0;
      int R_free_idx = 0;
      for (size_t i = 0; i < G->rowsize; i++){
        if (ddf_get_d(G->matrix[i][0])==0){
          if (set_member(i+1,G->linset)){
            for (size_t j = 0; j < A_eq.cols(); j++){
              R_free(j,R_free_idx) = ddf_get_d(G->matrix[i][1+j]);
            }
            R_free_idx++;
          }else{
            for (size_t j = 0; j < A_eq.cols(); j++){
              R_nonneg(j,R_nonneg_idx) = ddf_get_d(G->matrix[i][1+j]);
            }
            R_nonneg_idx++;
          }
        }else{
          for (size_t j = 0; j < A_eq.cols(); j++){
            V(j,V_idx) = ddf_get_d(G->matrix[i][1+j]);
          }
          V_idx++;
        }
      }

      // free
      ddf_FreeMatrix(A);
      ddf_FreeMatrix(G);
      ddf_FreePolyhedra(poly);
      return true;
    }else{
      if (verbose){
        ddf_WriteErrorMessages(stderr,err);
      }

      ddf_FreeMatrix(A);
      ddf_FreePolyhedra(poly);

      return false;
    }
  }

  bool HtoVgmp (const Eigen::MatrixXd& A_eq,
                const Eigen::VectorXd& b_eq,
                const Eigen::MatrixXd& A_ineq,
                const Eigen::VectorXd& b_ineq,
                Eigen::MatrixXd& V,
                Eigen::MatrixXd& R_nonneg,
                Eigen::MatrixXd& R_free,
                bool verbose,
                unsigned long int denom){
    if(!initialized) dd_set_global_constants();

    if( (A_eq.rows() != b_eq.rows()) || (A_ineq.rows() != b_ineq.rows()) || (A_eq.cols() != A_ineq.cols())) {
      std::cerr << "[cddeigen::HtoV] dimention mismatch" << std::endl;
      return false;
    }

    // setup A
    dd_MatrixPtr A = dd_CreateMatrix(A_eq.rows() + A_ineq.rows(),A_eq.cols()+1);
    for (size_t i = 0; i < A_eq.rows(); i++){
      set_addelem(A->linset,i+1);

      dd_set_si2(A->matrix[i][0], b_eq[i]*denom,denom);
      for (size_t j = 0; j < A_eq.cols(); j++){
        dd_set_si2(A->matrix[i][j+1], A_eq(i,j)*denom,denom);
      }
    }
    for (size_t i = 0; i < A_ineq.rows(); i++){
      dd_set_si2(A->matrix[A_eq.rows()+i][0], b_ineq[i]*denom,denom);
      for (size_t j = 0; j < A_ineq.cols(); j++){
        dd_set_si2(A->matrix[A_eq.rows()+i][j+1], A_ineq(i,j)*denom,denom);
      }
    }
    A->representation=dd_Inequality;

    if (verbose){
      std::cerr << "input H-representation:" << std::endl;
      dd_WriteMatrix(stderr,A);
      std::cerr << std::endl;
    }

    dd_rowindex newpos;
    dd_rowset impl_linset,redset;
    dd_ErrorType errf;
    dd_MatrixCanonicalize(&A, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      std::cerr << "after canonicalize:" << std::endl;
      dd_WriteMatrix(stderr,A);
      std::cerr << std::endl;
    }

    // compute the second (generator) representation
    dd_ErrorType err;
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(A, &err);
    // dd_PolyhedraPtr poly;
    // for(size_t i=0;i<10;i++){
    //   poly=dd_DDMatrix2Poly(A, &err);
    //   if(err != dd_NumericallyInconsistent) break;
    //   for(size_t i=0;i<A->rowsize;i++){
    //     for(size_t j=1;j<A->colsize;j++){
    //       if(dd_get_d(A->matrix[i][j]) != 0)
    //         dd_set_si2(A->matrix[i][j],(dd_get_d(A->matrix[i][j])+1e-8)*denom,denom);
    //     }
    //   }
    // }

    if (err==dd_NoError){
      dd_MatrixPtr G = dd_CopyGenerators(poly);

      if (verbose){
        std::cerr <<"output V-representation:" <<std::endl;;
        dd_WriteMatrix(stdout,G);
        std::cerr << std::endl;
      }

      // get matrix size
      int V_cols = 0;
      int R_nonneg_cols = 0;
      int R_free_cols = 0;
      for (size_t i = 0; i < G->rowsize; i++){
        if (dd_get_d(G->matrix[i][0]) ==0 ){
          if (set_member(i+1,G->linset)) R_free_cols++;
          else R_nonneg_cols++;
        }else V_cols++;
      }

      // create return matrix
      V.resize(A_eq.cols(),V_cols);
      R_nonneg.resize(A_eq.cols(),R_nonneg_cols);
      R_free.resize(A_eq.cols(),R_free_cols);
      int V_idx = 0;
      int R_nonneg_idx = 0;
      int R_free_idx = 0;
      for (size_t i = 0; i < G->rowsize; i++){
        if (dd_get_d(G->matrix[i][0])==0){
          if (set_member(i+1,G->linset)){
            for (size_t j = 0; j < A_eq.cols(); j++){
              R_free(j,R_free_idx) = dd_get_d(G->matrix[i][1+j]);
            }
            R_free_idx++;
          }else{
            for (size_t j = 0; j < A_eq.cols(); j++){
              R_nonneg(j,R_nonneg_idx) = dd_get_d(G->matrix[i][1+j]);
            }
            R_nonneg_idx++;
          }
        }else{
          for (size_t j = 0; j < A_eq.cols(); j++){
            V(j,V_idx) = dd_get_d(G->matrix[i][1+j]);
          }
          V_idx++;
        }
      }std::cerr<<"V"<<std::endl;std::cerr<<V<<std::endl;

      // free
      dd_FreeMatrix(A);
      dd_FreeMatrix(G);
      dd_FreePolyhedra(poly);
      return true;
    }else{
      if (verbose){
        dd_WriteErrorMessages(stderr,err);
      }

      dd_FreeMatrix(A);
      dd_FreePolyhedra(poly);

      return false;
    }
  }

  bool VtoH (const Eigen::MatrixXd& V,
             const Eigen::MatrixXd& R_nonneg,
             const Eigen::MatrixXd& R_free,
             Eigen::MatrixXd& A_eq,
             Eigen::VectorXd& b_eq,
             Eigen::MatrixXd& A_ineq,
             Eigen::VectorXd& b_ineq,
             bool verbose){
    if(!initialized) ddf_set_global_constants();

    if( (V.rows() != R_nonneg.rows()) || (R_nonneg.rows() != R_free.rows()) ) {
      std::cerr << "[cddeigen::VtoH] dimention mismatch" << std::endl;
      return false;
    }

    // setup G
    ddf_MatrixPtr G=ddf_CreateMatrix(V.cols()+R_nonneg.cols()+R_free.cols(),V.rows()+1);
    for (size_t i = 0; i < V.cols(); i++){
      ddf_set_si(G->matrix[i][0],1);
      for (size_t j = 0; j < V.rows(); j++){
        ddf_set_d(G->matrix[i][j+1],V(j,i));
      }
    }
    for (size_t i = 0; i < R_nonneg.cols(); i++){
      ddf_set_si(G->matrix[V.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        ddf_set_d(G->matrix[V.cols()+i][j+1],R_nonneg(j,i));
      }
    }
    for (size_t i = 0; i < R_free.cols(); i++){
      set_addelem(G->linset,V.cols()+R_nonneg.cols()+i+1);
      ddf_set_si(G->matrix[V.cols()+R_nonneg.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        ddf_set_d(G->matrix[V.cols()+R_nonneg.cols()+i][j+1],R_free(j,i));
      }
    }
    G->representation=ddf_Generator;

    if (verbose){
      std::cerr << "input V-representation:" << std::endl;;
      ddf_WriteMatrix(stderr,G);
      std::cerr << std::endl;
    }

    ddf_rowindex newpos;
    ddf_rowset impl_linset,redset;
    ddf_ErrorType errf;
    ddf_MatrixCanonicalize(&G, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      std::cerr << "after canonicalize:" <<std::endl;;
      ddf_WriteMatrix(stderr,G);
      std::cerr << std::endl;
    }

    // compute the second (generator) representation
    ddf_ErrorType err;
    ddf_PolyhedraPtr poly=ddf_DDMatrix2Poly(G, &err);

    if (err==ddf_NoError){
      ddf_MatrixPtr A = ddf_CopyInequalities(poly);

      if (verbose){
        std::cerr << "output H-representation:" << std::endl;;
        ddf_WriteMatrix(stderr,A);
        std::cerr << std::endl;
      }

      // get matrix size
      int A_eq_rows = 0;
      int A_ineq_rows = 0;
      for (size_t i = 0; i < A->rowsize; i++){
        if (set_member(i+1,A->linset)) A_eq_rows++;
        else A_ineq_rows++;
      }

      // create return matrix
      A_eq.resize(A_eq_rows,V.rows());
      b_eq.resize(A_eq_rows);
      A_ineq.resize(A_ineq_rows,V.rows());
      b_ineq.resize(A_ineq_rows);
      int A_eq_idx = 0;
      int A_ineq_idx = 0;
      for (size_t i = 0; i < A->rowsize; i++){
        if (set_member(i+1,A->linset)){
          for (size_t j = 0; j < V.rows(); j++){
            A_eq(A_eq_idx,j) = ddf_get_d(A->matrix[i][1+j]);
          }
          b_eq[A_eq_idx] = ddf_get_d(A->matrix[i][0]);
          A_eq_idx++;
        }else{
          for (size_t j = 0; j < V.rows(); j++){
            A_ineq(A_ineq_idx,j) = ddf_get_d(A->matrix[i][1+j]);
          }
          b_ineq[A_ineq_idx] = ddf_get_d(A->matrix[i][0]);
          A_ineq_idx++;
        }
      }

      // free
      ddf_FreeMatrix(A);
      ddf_FreeMatrix(G);
      ddf_FreePolyhedra(poly);

      return true;
    }else{
      if (verbose){
        ddf_WriteErrorMessages(stderr,err);
      }

      ddf_FreeMatrix(G);
      ddf_FreePolyhedra(poly);

      return false;
    }
  }

  bool VtoHgmp (const Eigen::MatrixXd& V,
                const Eigen::MatrixXd& R_nonneg,
                const Eigen::MatrixXd& R_free,
                Eigen::MatrixXd& A_eq,
                Eigen::VectorXd& b_eq,
                Eigen::MatrixXd& A_ineq,
                Eigen::VectorXd& b_ineq,
                bool verbose,
                unsigned long int denom){
    if(!initialized) dd_set_global_constants();

    if( (V.rows() != R_nonneg.rows()) || (R_nonneg.rows() != R_free.rows()) ) {
      std::cerr << "[cddeigen::VtoH] dimention mismatch" << std::endl;
      return false;
    }

    // setup G
    dd_MatrixPtr G=dd_CreateMatrix(V.cols()+R_nonneg.cols()+R_free.cols(),V.rows()+1);
    for (size_t i = 0; i < V.cols(); i++){
      dd_set_si(G->matrix[i][0],1);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_si2(G->matrix[i][j+1],V(j,i)*denom,denom);
      }
    }
    for (size_t i = 0; i < R_nonneg.cols(); i++){
      dd_set_si(G->matrix[V.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_si2(G->matrix[V.cols()+i][j+1],R_nonneg(j,i)*denom,denom);
      }
    }
    for (size_t i = 0; i < R_free.cols(); i++){
      set_addelem(G->linset,V.cols()+R_nonneg.cols()+i+1);
      dd_set_si(G->matrix[V.cols()+R_nonneg.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_si2(G->matrix[V.cols()+R_nonneg.cols()+i][j+1],R_free(j,i)*denom,denom);
      }
    }
    G->representation=dd_Generator;

    if (verbose){
      std::cerr << "input V-representation:" << std::endl;;
      dd_WriteMatrix(stderr,G);
      std::cerr << std::endl;
    }

    dd_rowindex newpos;
    dd_rowset impl_linset,redset;
    dd_ErrorType errf;
    dd_MatrixCanonicalize(&G, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      std::cerr << "after canonicalize:" <<std::endl;;
      dd_WriteMatrix(stderr,G);
      std::cerr << std::endl;
    }

    // compute the second (generator) representation
    dd_ErrorType err;
    dd_PolyhedraPtr poly = dd_DDMatrix2Poly(G, &err);
    // dd_PolyhedraPtr poly;
    // for(size_t i=0;i<10;i++){
    //   poly=dd_DDMatrix2Poly(G, &err);
    //   if(err != dd_NumericallyInconsistent) break;
    //   for(size_t i=0;i<G->rowsize;i++){
    //     for(size_t j=1;j<G->colsize;j++){
    //       if(dd_get_d(G->matrix[i][j]) != 0)
    //         dd_set_si2(G->matrix[i][j],(dd_get_d(G->matrix[i][j])+1e-8)*denom,denom);
    //     }
    //   }
    // }

    if (err==dd_NoError){
      dd_MatrixPtr A = dd_CopyInequalities(poly);

      if (verbose){
        std::cerr << "output H-representation:" << std::endl;
        dd_WriteMatrix(stderr,A);
        std::cerr << std::endl;
      }

      // get matrix size
      int A_eq_rows = 0;
      int A_ineq_rows = 0;
      for (size_t i = 0; i < A->rowsize; i++){
        if (set_member(i+1,A->linset)) A_eq_rows++;
        else A_ineq_rows++;
      }

      // create return matrix
      A_eq.resize(A_eq_rows,V.rows());
      b_eq.resize(A_eq_rows);
      A_ineq.resize(A_ineq_rows,V.rows());
      b_ineq.resize(A_ineq_rows);
      int A_eq_idx = 0;
      int A_ineq_idx = 0;
      for (size_t i = 0; i < A->rowsize; i++){
        if (set_member(i+1,A->linset)){
          for (size_t j = 0; j < V.rows(); j++){
            A_eq(A_eq_idx,j) = dd_get_d(A->matrix[i][1+j]);
          }
          b_eq[A_eq_idx] = dd_get_d(A->matrix[i][0]);
          A_eq_idx++;
        }else{
          for (size_t j = 0; j < V.rows(); j++){
            A_ineq(A_ineq_idx,j) = dd_get_d(A->matrix[i][1+j]);
          }
          b_ineq[A_ineq_idx] = dd_get_d(A->matrix[i][0]);
          A_ineq_idx++;
        }
      }

      // free
      dd_FreeMatrix(A);
      dd_FreeMatrix(G);
      dd_FreePolyhedra(poly);

      return true;
    }else{
      if (verbose){
        dd_WriteErrorMessages(stderr,err);
      }

      dd_FreeMatrix(G);
      dd_FreePolyhedra(poly);

      return false;
    }
  }

}
