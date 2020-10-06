#include <cddeigen/cddeigen.h>
#include <iostream>

namespace cddeigen{
  static bool initialized=false;

  //ddをddfに変えるとgmpを使わなくなる
  //が，今はcddをリンクしているのでgmpを使っていない

  bool HtoVgmp (const Eigen::MatrixXd& A_eq,
                const Eigen::VectorXd& b_eq,
                const Eigen::MatrixXd& A_ineq,
                const Eigen::VectorXd& b_ineq,
                Eigen::MatrixXd& V,
                Eigen::MatrixXd& R_nonneg,
                Eigen::MatrixXd& R_free,
                bool verbose){
    if(!initialized) dd_set_global_constants();

    if( (A_eq.rows() != b_eq.rows()) || (A_ineq.rows() != b_ineq.rows()) || (A_eq.cols() != A_ineq.cols())) {
      std::cerr << "[cddeigen::HtoV] dimention mismatch" << std::cerr;
      return false;
    }

    // setup A
    dd_MatrixPtr A = dd_CreateMatrix(A_eq.rows() + A_ineq.rows(),A_eq.cols()+1);
    for (size_t i = 0; i < A_eq.rows(); i++){
      set_addelem(A->linset,i+1);

      dd_set_d(A->matrix[i][0], b_eq[i]);
      for (size_t j = 0; j < A_eq.cols(); j++){
        dd_set_d(A->matrix[i][j+1], A_eq(i,j));
      }
    }
    for (size_t i = 0; i < A_ineq.rows(); i++){
      dd_set_d(A->matrix[A_eq.rows()+i][0], b_ineq[i]);
      for (size_t j = 0; j < A_ineq.cols(); j++){
        dd_set_d(A->matrix[A_eq.rows()+i][j+1], A_ineq(i,j));
      }
    }
    A->representation=dd_Inequality;

    if (verbose){
      printf("\ninput H-representation:\n");
      dd_WriteMatrix(stderr,A);  printf("\n");
    }

    dd_rowindex newpos;
    dd_rowset impl_linset,redset;
    dd_ErrorType errf;
    dd_MatrixCanonicalize(&A, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      printf("\nafter canonicalize:\n");
      dd_WriteMatrix(stderr,A);  printf("\n");
    }

    // compute the second (generator) representation
    dd_ErrorType err;
    dd_PolyhedraPtr poly=dd_DDMatrix2Poly(A, &err);

    if (err==dd_NoError){
      dd_MatrixPtr G = dd_CopyGenerators(poly);

      if (verbose){
        printf("\noutput V-representation:\n");
        dd_WriteMatrix(stdout,G);
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
            for (size_t j = 0; j < A_eq.rows(); j++){
              R_nonneg(j,R_nonneg_idx) = dd_get_d(G->matrix[i][1+j]);
            }
            R_nonneg_idx++;
          }
        }else{
          for (size_t j = 0; j < A_eq.rows(); j++){
            V(j,V_idx) = dd_get_d(G->matrix[i][1+j]);
          }
          V_idx++;
        }
      }

      // free
      dd_FreeMatrix(A);
      dd_FreeMatrix(G);
      dd_FreePolyhedra(poly);
      return true;
    }else{
      if (verbose){
        dd_WriteErrorMessages(stdout,err);
      }

      dd_FreeMatrix(A);
      dd_FreePolyhedra(poly);

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
                bool verbose){
    if(!initialized) dd_set_global_constants();

    if( (V.rows() != R_nonneg.rows()) || (R_nonneg.rows() != R_free.rows()) ) {
      std::cerr << "[cddeigen::VtoH] dimention mismatch" << std::cerr;
      return false;
    }

    // setup G
    dd_MatrixPtr G=dd_CreateMatrix(V.cols()+R_nonneg.cols()+R_free.cols(),V.rows()+1);
    for (size_t i = 0; i < V.cols(); i++){
      dd_set_si(G->matrix[i][0],1);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_d(G->matrix[i][j+1],V(j,i));
      }
    }
    for (size_t i = 0; i < R_nonneg.cols(); i++){
      dd_set_si(G->matrix[V.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_d(G->matrix[V.cols()+i][j+1],R_nonneg(j,i));
      }
    }
    for (size_t i = 0; i < R_free.cols(); i++){
      set_addelem(G->linset,V.cols()+R_nonneg.cols()+i+1);
      dd_set_si(G->matrix[V.cols()+R_nonneg.cols()+i][0],0);
      for (size_t j = 0; j < V.rows(); j++){
        dd_set_d(G->matrix[V.cols()+R_nonneg.cols()+i][j+1],R_free(j,i));
      }
    }
    G->representation=dd_Generator;

    if (verbose){
      printf("\ninput V-representation:\n");
      dd_WriteMatrix(stderr,G);  printf("\n");
    }

    dd_rowindex newpos;
    dd_rowset impl_linset,redset;
    dd_ErrorType errf;
    dd_MatrixCanonicalize(&G, &impl_linset, &redset, &newpos, &errf);

    if (verbose){
      printf("\nafter canonicalize:\n");
      dd_WriteMatrix(stderr,G);  printf("\n");
    }

    // compute the second (generator) representation
    dd_ErrorType err;
    dd_PolyhedraPtr poly=dd_DDMatrix2Poly(G, &err);

    if (err==dd_NoError){
      dd_MatrixPtr A = dd_CopyInequalities(poly);

      if (verbose){
        printf("\noutput H-representation:\n");
        dd_WriteMatrix(stdout,A);
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
        dd_WriteErrorMessages(stdout,err);
      }

      dd_FreeMatrix(G);
      dd_FreePolyhedra(poly);

      return false;
    }
  }

}
