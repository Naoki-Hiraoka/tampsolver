#include <cddeigen/cddeigen.h>
#include <iostream>

namespace cddeigen{
  static bool initialized=false;

  bool HtoV (const Eigen::MatrixXd& A_eq,
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

    ddf_rowindex newpos;
    ddf_rowset impl_linset,redset;
    ddf_ErrorType errf;
    ddf_MatrixCanonicalize(&A, &impl_linset, &redset, &newpos, &errf);

    // compute the second (generator) representation
    ddf_ErrorType err;
    ddf_PolyhedraPtr poly=ddf_DDMatrix2Poly(A, &err);

    if (err==ddf_NoError){
      ddf_MatrixPtr G = ddf_CopyGenerators(poly);

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
      V = Eigen::MatrixXd(A_eq.cols(),V_cols);
      R_nonneg = Eigen::MatrixXd(A_eq.cols(),R_nonneg_cols);
      R_free = Eigen::MatrixXd(A_eq.cols(),R_free_cols);
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
            for (size_t j = 0; j < A_eq.rows(); j++){
              R_nonneg(j,R_nonneg_idx) = ddf_get_d(G->matrix[i][1+j]);
            }
            R_nonneg_idx++;
          }
        }else{
          for (size_t j = 0; j < A_eq.rows(); j++){
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
        ddf_WriteErrorMessages(stdout,err);
      }

      ddf_FreeMatrix(A);
      ddf_FreePolyhedra(poly);

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
    if(!initialized) dd_set_global_constants();

    if( (V.rows() != R_nonneg.rows()) || (R_nonneg.rows() != R_free.rows()) ) {
      std::cerr << "[cddeigen::VtoH] dimention mismatch" << std::cerr;
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

    ddf_rowindex newpos;
    ddf_rowset impl_linset,redset;
    ddf_ErrorType errf;
    ddf_MatrixCanonicalize(&G, &impl_linset, &redset, &newpos, &errf);

    // compute the second (generator) representation
    ddf_ErrorType err;
    ddf_PolyhedraPtr poly=ddf_DDMatrix2Poly(G, &err);

    if (err==ddf_NoError){
      ddf_MatrixPtr A = ddf_CopyInequalities(poly);

      // get matrix size
      int A_eq_rows = 0;
      int A_ineq_rows = 0;
      for (size_t i = 0; i < A->rowsize; i++){
        if (set_member(i+1,A->linset)) A_eq_rows++;
        else A_ineq_rows++;
      }

      // create return matrix
      A_eq = Eigen::MatrixXd(A_eq_rows,V.rows());
      b_eq = Eigen::VectorXd(A_eq_rows);
      A_ineq = Eigen::MatrixXd(A_ineq_rows,V.rows());
      b_ineq = Eigen::VectorXd(A_ineq_rows);
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
        ddf_WriteErrorMessages(stdout,err);
      }

      ddf_FreeMatrix(G);
      ddf_FreePolyhedra(poly);

      return false;
    }
  }

}
