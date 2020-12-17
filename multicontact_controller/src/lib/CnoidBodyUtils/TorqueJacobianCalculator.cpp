#include <multicontact_controller/lib/CnoidBodyUtils/TorqueJacobianCalculator.h>

#include <cnoid/EigenUtil>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    TorqueJacobianCalculator::TorqueJacobianCalculator(cnoid::Body* robot)
      : robot_(robot),
        Dg_(6+robot_->numJoints(),6+robot_->numJoints()),
        DJw_(6+robot_->numJoints(),6+robot_->numJoints()),
        subMasses_(robot_->numLinks())
    {
      // construct rootaxis
      rootAxis_.push_back(cnoid::Vector3::UnitX());
      rootAxis_.push_back(cnoid::Vector3::UnitY());
      rootAxis_.push_back(cnoid::Vector3::UnitZ());
      rootAxis_.push_back(cnoid::Vector3::UnitX());
      rootAxis_.push_back(cnoid::Vector3::UnitY());
      rootAxis_.push_back(cnoid::Vector3::UnitZ());

      // construct relationMap
      this->generateRelationMap();
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& TorqueJacobianCalculator::calcDg(const cnoid::Vector3 g){
      this->calcSubMass(robot_->rootLink());

      for(size_t i=3; i<6; i++){
        for(size_t j=0; j<3; j++){
          Dg_.coeffRef(i,j) = g.dot(cnoid::hat(rootAxis_[i]) * rootAxis_[j] * subMasses_[robot_->rootLink()->index()].m);
        }
      }

      for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 3 ; j < 6; j++){
          Dg_.coeffRef(i,j) = g.dot(cnoid::hat(rootAxis_[i]) * cnoid::hat(rootAxis_[j]) * (subMasses_[robot_->rootLink()->index()].mwc - robot_->rootLink()->p() * subMasses_[robot_->rootLink()->index()].m));
        }
      }

      for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 0 ; j < robot_->numJoints(); j++){//j = j+6
          cnoid::Link* joint_j = robot_->joint(j);
          const SubMass& sub_j = subMasses_[joint_j->index()];
          Dg_.coeffRef(i,6+j) = g.dot(cnoid::hat(rootAxis_[i]) * cnoid::hat(joint_j->R() * joint_j->a()) * (sub_j.mwc - joint_j->p() * sub_j.m));
        }
      }

      for(size_t i = 0 ; i < robot_->numJoints(); i++){//i = i+6
        for(size_t j = 3 ; j < 6; j++){
          cnoid::Link* joint_i = robot_->joint(i);
          const SubMass& sub_i = subMasses_[joint_i->index()];
          Dg_.coeffRef(6+i,j) = g.dot(cnoid::hat(rootAxis_[j]) * cnoid::hat(joint_i->R() * joint_i->a()) * (sub_i.mwc - joint_i->p() * sub_i.m));
        }
      }

      for(size_t i = 0; i < robot_->numJoints(); i++){
        for(size_t j = 0; j < robot_->numJoints(); j++){
          cnoid::Link* joint_i = robot_->joint(i);
          const SubMass& sub_i = subMasses_[joint_i->index()];
          cnoid::Link* joint_j = robot_->joint(j);
          const SubMass& sub_j = subMasses_[joint_j->index()];

          switch(relationMap_[joint_i][joint_j]){
          case Relation::OTHER_PATH : //(if i, j are in different path)
            break;
          case Relation::SAME_JOINT : //(if i=j)
          case Relation::ANCESTOR : //(if root->i->j)
            Dg_.coeffRef(6+i,6+j) = g.dot(cnoid::hat(joint_i->R() * joint_i->a()) * cnoid::hat(joint_j->R() * joint_j->a()) * (sub_j.mwc - joint_j->p() * sub_j.m));
            break;
          case Relation::DESCENDANT : //(if root->j->i)
            Dg_.coeffRef(6+i,6+j) = g.dot(cnoid::hat(joint_j->R() * joint_j->a()) * cnoid::hat(joint_i->R() * joint_i->a()) * (sub_i.mwc - joint_i->p() * sub_i.m));
            break;
          default:
            break;
          }
        }
      }

      return Dg_;
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>&  TorqueJacobianCalculator::calcDJw(std::vector<std::shared_ptr<ContactPoint> > contactPoints){
      std::vector<cnoid::Vector3> f_worlds;//world系
      std::vector<cnoid::Vector3> n_worlds;//world系,contactpointまわり
      std::vector<cnoid::Position> T_worlds;//world系
      for(size_t i=0;i<contactPoints.size();i++){
        const cnoid::Position T = contactPoints[i]->parent()->T() * contactPoints[i]->T_local();
        const cnoid::Matrix3 R = T.linear();

        f_worlds.push_back(R * contactPoints[i]->F().head<3>());
        n_worlds.push_back(R * contactPoints[i]->F().tail<3>());
        T_worlds.push_back(T);
      }

      for(size_t i=3; i<6; i++){
        for(size_t j=0; j<3; j++){
          double value = 0;
          for(size_t m = 0; m < contactPoints.size(); m++){
            value += f_worlds[m].dot(cnoid::hat(rootAxis_[i]) * rootAxis_[j]);
          }
          DJw_.coeffRef(i,j) = value;
        }
      }

      for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 3 ; j < 6; j++){
            double value = 0;
            for(size_t m = 0; m < contactPoints.size(); m++){
              value += f_worlds[m].dot(cnoid::hat(rootAxis_[i]) * cnoid::hat(rootAxis_[j]) * (T_worlds[m].translation() - robot_->rootLink()->p()));
            }
            DJw_.coeffRef(i,j) = value;
        }
      }

      for(size_t i = 3 ; i < 6; i++){
        for(size_t j = 0 ; j < robot_->numJoints(); j++){//j = j+6
          cnoid::Link* joint_j = robot_->joint(j);
          double value = 0;
          bool changed = false;
          for(size_t m = 0; m < contactPoints.size(); m++){
            cnoid::Link* joint_m = contactPoints[m]->parent();
            switch(relationMap_[joint_j][joint_m]){
            case Relation::OTHER_PATH : //(if j, m are in different path)
              break;
            case Relation::SAME_JOINT : //(if j=m)
            case Relation::ANCESTOR : //(if root->j->m)
              value += f_worlds[m].dot(cnoid::hat(rootAxis_[i]) * cnoid::hat(joint_j->R() * joint_j->a()) * (T_worlds[m].translation() - joint_j->p()));
              changed = true;
              break;
            case Relation::DESCENDANT : //(if root->m->j)
              break;
            default:
              break;
            }
          }
          if(changed)DJw_.coeffRef(i,6+j) = value;
        }
      }

      for(size_t i = 0 ; i < robot_->numJoints(); i++){//i = i+6
        for(size_t j = 3 ; j < 6; j++){
          cnoid::Link* joint_i = robot_->joint(i);
          const SubMass& sub_i = subMasses_[joint_i->index()];
          double value;
          bool changed = false;
          for(size_t m = 0; m < contactPoints.size(); m++){
            cnoid::Link* joint_m = contactPoints[m]->parent();
            switch(relationMap_[joint_i][joint_m]){
            case Relation::OTHER_PATH : //(if i, m are in different path)
              break;
            case Relation::SAME_JOINT : //(if i=m)
            case Relation::ANCESTOR : //(if root->i->m)
              value += f_worlds[m].dot( cnoid::hat(rootAxis_[j]) * cnoid::hat(joint_i->R() * joint_i->a()) * (T_worlds[m].translation() - joint_i->p()));
              value += n_worlds[m].dot( cnoid::hat(rootAxis_[j]) * joint_i->R() * joint_i->a());
              changed = true;
              break;
            case Relation::DESCENDANT: //(if root->m->i)
              break;
            default:
              break;
            }
          }
          if(changed)DJw_.coeffRef(6+i,j) = value;
        }
      }

      for(size_t i = 0; i < robot_->numJoints(); i++){
        for(size_t j = 0; j < robot_->numJoints(); j++){
          cnoid::Link* joint_i = robot_->joint(i);
          const SubMass& sub_i = subMasses_[joint_i->index()];
          cnoid::Link* joint_j = robot_->joint(j);
          const SubMass& sub_j = subMasses_[joint_j->index()];

          double value = 0;
          bool changed = false;
          switch(relationMap_[joint_i][joint_j]){
          case Relation::OTHER_PATH : //(if i, j are in different path)
            break;
          case Relation::SAME_JOINT : //(if i=j)
          case Relation::ANCESTOR : //(if root->i->j)
            for(size_t m = 0; m < contactPoints.size(); m++){
              cnoid::Link* joint_m = contactPoints[m]->parent();
              switch(relationMap_[joint_j][joint_m]){
              case Relation::OTHER_PATH : //(if j, m are in different path)
                break;
              case Relation::SAME_JOINT : //(if j=m)
              case Relation::ANCESTOR : //(if root->j->m)
                value += f_worlds[m].dot( cnoid::hat(joint_i->R() * joint_i->a()) * cnoid::hat(joint_j->R() * joint_j->a()) * (T_worlds[m].translation() - joint_j->p()));
                changed = true;
                break;
              case Relation::DESCENDANT : //(if root->m->j)
                break;
              default:
                break;
              }
            }
            break;
          case Relation::DESCENDANT : //(if root->j->i)
            for(size_t m = 0; m < contactPoints.size(); m++){
              cnoid::Link* joint_m = contactPoints[m]->parent();
              switch(relationMap_[joint_i][joint_m]){
              case Relation::OTHER_PATH : //(if i, m are in different path)
                break;
              case Relation::SAME_JOINT : //(if i=m)
              case Relation::ANCESTOR : //(if root->i->m)
                value += f_worlds[m].dot( cnoid::hat(joint_j->R() * joint_j->a()) * cnoid::hat(joint_i->R() * joint_i->a()) * (T_worlds[m].translation() - joint_i->p()));
                value += n_worlds[m].dot( cnoid::hat(joint_j->R() * joint_j->a()) * joint_i->R() * joint_i->a());
                changed = true;
                break;
              case Relation::DESCENDANT : //(if root->m->i)
                break;
              default:
                break;
              }
            }
            break;
          default:
            break;
          }

          if(changed) DJw_.coeffRef(6+i,6+j) = value;
        }
      }

      return DJw_;
    }

    // copied from Choreonoid/Body/Jacobian.cpp
    void TorqueJacobianCalculator::calcSubMass(cnoid::Link* link){
      cnoid::Matrix3 R = link->R();
      SubMass& sub = subMasses_[link->index()];
      sub.m = link->m();
      sub.mwc = link->m() * link->wc();

      for(cnoid::Link* child = link->child(); child; child = child->sibling()){
        calcSubMass(child);
        SubMass& childSub = subMasses_[child->index()];
        sub.m += childSub.m;
        sub.mwc += childSub.mwc;
      }
    }

    bool TorqueJacobianCalculator::generateRelationMap(){
      for(size_t i = 0; i < robot_->numJoints(); i++){
        cnoid::Link* joint_i = robot_->joint(i);
        std::map<cnoid::Link*, Relation> relationMap_i;
        for(size_t j = 0; j < robot_->numJoints(); j++){
          cnoid::Link* joint_j = robot_->joint(j);

          if(joint_i == joint_j) {
            relationMap_i[joint_j] = Relation::SAME_JOINT;
            continue;
          }

          bool found = false;
          for(cnoid::Link* i_ancestor = joint_i->parent(); i_ancestor ; i_ancestor = i_ancestor->parent()){
            if(i_ancestor == joint_j){
              relationMap_i[joint_j] = Relation::DESCENDANT;
              found = true;
              break;
            }
          }
          if(found) continue;

          for(cnoid::Link* j_ancestor = joint_j->parent(); j_ancestor ; j_ancestor = j_ancestor->parent()){
            if(j_ancestor == joint_i){
              relationMap_i[joint_j] = Relation::ANCESTOR;
              found = true;
              break;
            }
          }
          if(found) continue;

          relationMap_i[joint_j] = Relation::OTHER_PATH;
        }
        relationMap_[joint_i] = relationMap_i;
      }

    }

  };
};
