#include "SCFRConstraint.h"

namespace IK{
  SCFRConstraint::SCFRConstraint(cnoid::Body* _robot, const std::vector<std::shared_ptr<RobotConfig::EndEffector> >& _endeffectors):
    robot(_robot),
    endeffectors(_endeffectors),
    initial_p(true),
    g(9.80665)
  {
  }

  Eigen::VectorXd SCFRConstraint::calc_minineq (){
    return Eigen::VectorXd();
  }

  Eigen::VectorXd SCFRConstraint::calc_maxineq (){
    return Eigen::VectorXd();
  }

  Eigen::SparseMatrix<double,Eigen::RowMajor> SCFRConstraint::calc_jacobianineq (const std::vector<cnoid::Body*>& bodies) {
    int dim = 0;
    for(size_t i=0;i<bodies.size();i++){
      dim += 6 + bodies[i]->numJoints();
    }

    if(this->initial_p){
      // SCFRを計算
      // x = [px py w1 w2 ...]^T
      // wはエンドエフェクタ座標系．エンドエフェクタまわり

      // Grasp Matrix Gx = h
      Eigen::SparseMatrix<double,Eigen::RowMajor> G(6,2+6*this->endeffectors.size());
      Eigen::VectorXd h=Eigen::VectorXd::Zero(6);
      std::vector<Eigen::Triplet<double> > G_tripletList;
      G_tripletList.reserve(6*6*this->endeffectors.size());
      G_tripletList.push_back(Eigen::Triplet<double>(3,1,-this->robot->mass()*this->g));
      G_tripletList.push_back(Eigen::Triplet<double>(4,0,this->robot->mass()*this->g));
      h[2] = this->robot->mass()*this->g;
      for (size_t i=0;i<this->endeffectors.size();i++){
        const cnoid::Position pos = this->endeffectors[i]->getlink()->T() * this->endeffectors[i]->getlocalpos();
        const cnoid::Matrix3& R = pos.linear();
        const cnoid::Matrix3& p_x_R = cnoid::hat(pos.translation()) * R;

        for(size_t row=0;row<3;row++){
          for(size_t col=0;col<3;col++){
            G_tripletList.push_back(Eigen::Triplet<double>(row,2+6*i+col,R(row,col)));
          }
        }
        for(size_t row=0;row<3;row++){
          for(size_t col=0;col<3;col++){
            G_tripletList.push_back(Eigen::Triplet<double>(3+row,2+6*i+col,p_x_R(row,col)));
          }
        }
        for(size_t row=0;row<3;row++){
          for(size_t col=0;col<3;col++){
            G_tripletList.push_back(Eigen::Triplet<double>(3+row,2+6*i+3+col,R(row,col)));
          }
        }
      }

      //接触力制約．Aw = b, Cw >= d
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > As(this->endeffectors.size());
      std::vector<Eigen::VectorXd> bs(this->endeffectors.size());
      std::vector<Eigen::SparseMatrix<double,Eigen::RowMajor> > Cs(this->endeffectors.size());
      std::vector<Eigen::VectorXd> ds(this->endeffectors.size());
      int num_eqs = 0;
      int num_ineqs = 0;
      for (size_t i=0;i<this->endeffectors.size();i++){
        this->endeffectors[i]->getcontact()->getContactConstraint(As[i],bs[i],Cs[i],ds[i]);
        num_eqs += bs[i].rows();
        num_ineqs += ds[i].rows();
      }
      Eigen::SparseMatrix<double,Eigen::RowMajor> A_appended(num_eqs,2+6*this->endeffectors.size());
      Eigen::VectorXd b_appended(num_eqs);
      Eigen::SparseMatrix<double,Eigen::RowMajor> C_appended(num_ineqs,2+6*this->endeffectors.size());
      Eigen::VectorXd d_appended(num_ineqs);
      std::vector<Eigen::Triplet<double> > A_tripletList;
      std::vector<Eigen::Triplet<double> > C_tripletList;
      A_tripletList.reserve(num_eqs*2);//適当
      C_tripletList.reserve(num_ineqs*2);//適当
      int idx_eq=0;
      int idx_ineq=0;
      for(size_t i=0;i<this->endeffectors.size();i++){
        for (int k=0; k < As[i].outerSize(); ++k){
          for (Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(As[i],k); it; ++it){
            A_tripletList.push_back(Eigen::Triplet<double>(idx_eq+it.row(),2+i*6+it.col(),it.value()));
          }
        }
        b_appended.segment(idx_eq,bs[i].rows()) = bs[i];
        idx_eq += bs[i].rows();

        for (int k=0; k < Cs[i].outerSize(); ++k){
          for (Eigen::SparseMatrix<double,Eigen::RowMajor>::InnerIterator it(Cs[i],k); it; ++it){
            C_tripletList.push_back(Eigen::Triplet<double>(idx_ineq+it.row(),2+i*6+it.col(),it.value()));
          }
        }
        d_appended.segment(idx_ineq,ds[i].rows()) = ds[i];
        idx_ineq += ds[i].rows();
      }
      A_appended.setFromTriplets(A_tripletList.begin(), A_tripletList.end());
      C_appended.setFromTriplets(C_tripletList.begin(), C_tripletList.end());

      // all. Ax = b, Cx >= d
      Eigen::SparseMatrix<double,Eigen::RowMajor> A(G.rows()+A_appended.rows(),2+6*this->endeffectors.size());
      A.middleRows(0,G.rows()) = G;
      A.middleRows(G.rows(),A_appended.rows()) = A_appended;
      Eigen::VectorXd b(h.rows()+b_appended.rows());
      b.segment(0,h.rows()) = h;
      b.segment(h.rows(),b_appended.rows()) = b_appended;
      Eigen::SparseMatrix<double,Eigen::RowMajor> C = C_appended;
      Eigen::VectorXd d = d_appended;

      // SCFRを計算
      this->calcSCFR(A,b,C,d);

      this->initial_p = false;
    }

    return Eigen::SparseMatrix<double,Eigen::RowMajor>(0,dim);
  }

  std::vector<cnoid::SgNodePtr> SCFRConstraint::getDrawOnObjects() {
    if(!this->SCFRlines){
      this->SCFRlines = new cnoid::SgLineSet;
      this->SCFRlines->setLineWidth(3.0);
      this->SCFRlines->getOrCreateColors()->resize(1);
      this->SCFRlines->getOrCreateColors()->at(0) = cnoid::Vector3f(0.0,1.0,0.0);

      this->SCFRlines->getOrCreateVertices()->resize(0);
      //SCFRlines->getOrCreateVertices()->resize(2);
      //SCFRlines->colorIndices().resize(0);
      //SCFRlines->addLine(0,1); lines->colorIndices().push_back(0); lines->colorIndices().push_back(0);
    }

    // update position


    std::vector<cnoid::SgNodePtr> ret{SCFRlines};

    for(size_t i=0;i<this->endeffectors.size();i++){
      std::vector<cnoid::SgNodePtr> contactlines = this->endeffectors[i]->getcontact()->getDrawOnObjects(this->endeffectors[i]->getlink()->T()*this->endeffectors[i]->getlocalpos());
      std::copy(contactlines.begin(),contactlines.end(),std::back_inserter(ret));
    }

    return ret;
  }
}
