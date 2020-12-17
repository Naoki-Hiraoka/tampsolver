#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <cnoid/EigenUtil>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    cnoid::Body* loadBodyFromParam(const std::string& paramname){
      ros::NodeHandle n;
      std::string vrml_file;
      if (!n.getParam(paramname, vrml_file)) {
        ROS_WARN("Failed to get param 'vrml_file'");
        return nullptr;
      }
      // package://に対応
      std::string packagestr = "package://";
      if(vrml_file.size()>packagestr.size() && vrml_file.substr(0,packagestr.size()) == packagestr){
        vrml_file = vrml_file.substr(packagestr.size());
        int pos = vrml_file.find("/");
        vrml_file = ros::package::getPath(vrml_file.substr(0,pos)) + vrml_file.substr(pos);
      }
      cnoid::BodyLoader loader;
      cnoid::Body* robot = loader.load(vrml_file);
      if(!robot){
        ROS_WARN("Failed to load %s", vrml_file.c_str());
        return nullptr;
      }

      return robot;
    }

    cnoid::Link* getLinkFromURDFlinkName(cnoid::Body* robot, const std::string& linkname){
      for(size_t i=0;i<robot->links().size();i++){
        cnoid::Affine3 tmp;
        cnoid::SgNodePath path = robot->links()[i]->visualShape()->findNode(linkname,tmp);
        if(path.size()!=0){
          return robot->links()[i];
        }
      }
      return nullptr;
    };

    void jointStateToBody(const sensor_msgs::JointState::ConstPtr& msg, cnoid::Body* robot){
      if(robot){
        for(size_t i=0;i<msg->name.size();i++){
          cnoid::Link* joint = robot->link(msg->name[i]);
          if(!joint) continue;
          if(msg->position.size() == msg->name.size()) joint->q() = msg->position[i];
          if(msg->velocity.size() == msg->name.size()) joint->dq() = msg->velocity[i];
          if(msg->effort.size() == msg->name.size()) joint->u() = msg->effort[i];
        }
        robot->calcForwardKinematics(false,false);
      }
    }

    void imuToBody(const sensor_msgs::Imu::ConstPtr& msg, cnoid::Body* robot){
      if(robot){
        // rootのvel, accがない TODO
        cnoid::Device* device = robot->findDevice(msg->header.frame_id);
        if(!device) return;
        cnoid::Matrix3 currentdeviceR = device->link()->R() * device->T_local().linear();
        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen(msg->orientation,q);
        cnoid::Matrix3 realdeviceR = q.normalized().toRotationMatrix();
        cnoid::Matrix3 nextrootR = realdeviceR * currentdeviceR.inverse() * robot->rootLink()->R();
        robot->rootLink()->R() = nextrootR;
        robot->calcForwardKinematics(false,false);
      }
    }

    cnoid::Matrix3 orientCoordsToAxis(const cnoid::Matrix3& coords, const cnoid::Vector3& axis/*local 系*/, const cnoid::Vector3& target_axis/*world系*/){
      Eigen::Vector3d rotate_axis = (coords*axis).cross(target_axis);
      if(rotate_axis.norm()==0){
        return coords;
      }else{
        double sin = rotate_axis.norm();
        double cos = (coords*axis).dot(target_axis);
        double angle = std::atan2(sin,cos);
        return Eigen::AngleAxisd(angle,rotate_axis.normalized()).toRotationMatrix() * coords;
      }
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPoint::calcJacobian(cnoid::Link* parent, cnoid::Position& T_local){//world系,contactpoint周り
      if(path_.empty() || path_.endLink() != parent){

        std::vector<Eigen::Triplet<double> > tripletList;
        tripletList.reserve(100);//適当

        //root 6dof
        for(size_t j=0;j<6;j++){
          tripletList.push_back(Eigen::Triplet<double>(j,j,1));
        }
        //  0     p[2] -p[1]
        // -p[2]  0     p[0]
        //  p[1] -p[0]  0
        tripletList.push_back(Eigen::Triplet<double>(0,4,1));
        tripletList.push_back(Eigen::Triplet<double>(0,5,1));
        tripletList.push_back(Eigen::Triplet<double>(1,3,1));
        tripletList.push_back(Eigen::Triplet<double>(1,5,1));
        tripletList.push_back(Eigen::Triplet<double>(2,3,1));
        tripletList.push_back(Eigen::Triplet<double>(2,4,1));

        //joints
        path_.setPath(parent);
        for(size_t j=0;j<path_.numJoints();j++){
          int col = 6+path_.joint(j)->jointId();
          tripletList.push_back(Eigen::Triplet<double>(0,col,1));
          tripletList.push_back(Eigen::Triplet<double>(1,col,1));
          tripletList.push_back(Eigen::Triplet<double>(2,col,1));
          tripletList.push_back(Eigen::Triplet<double>(3,col,1));
          tripletList.push_back(Eigen::Triplet<double>(4,col,1));
          tripletList.push_back(Eigen::Triplet<double>(5,col,1));
        }

        jacobian_ = Eigen::SparseMatrix<double,Eigen::RowMajor>(6,6+parent->body()->numJoints());
        jacobian_.setFromTriplets(tripletList.begin(), tripletList.end());
      }

      const cnoid::Position target_position = parent->T() * T_local;
      const cnoid::Vector3 target_p = target_position.translation();
      //root 6dof
      for(size_t j=0;j<6;j++){
        this->jacobian_.coeffRef(j,j) = 1;
      }
      cnoid::Vector3 dp = target_p - parent->body()->rootLink()->p();
      //  0     p[2] -p[1]
      // -p[2]  0     p[0]
      //  p[1] -p[0]  0
      jacobian_.coeffRef(0,4)=dp[2];
      jacobian_.coeffRef(0,5)=-dp[1];
      jacobian_.coeffRef(1,3)=-dp[2];
      jacobian_.coeffRef(1,5)=dp[0];
      jacobian_.coeffRef(2,3)=dp[1];
      jacobian_.coeffRef(2,4)=-dp[0];

      //joints
      for(size_t j=0;j<path_.numJoints();j++){
        int col = 6+path_.joint(j)->jointId();
        cnoid::Vector3 omega = path_.joint(j)->R() * path_.joint(j)->a();
        if(!path_.isJointDownward(j)) omega = -omega;
        if(path_.joint(j)->jointType() == cnoid::Link::JointType::PRISMATIC_JOINT ||
           path_.joint(j)->jointType() == cnoid::Link::JointType::SLIDE_JOINT){
          jacobian_.coeffRef(0,col)=omega[0];
          jacobian_.coeffRef(1,col)=omega[1];
          jacobian_.coeffRef(2,col)=omega[2];
          jacobian_.coeffRef(3,col)=0;
          jacobian_.coeffRef(4,col)=0;
          jacobian_.coeffRef(5,col)=0;
        }
        if(path_.joint(j)->jointType() == cnoid::Link::JointType::ROTATIONAL_JOINT ||
           path_.joint(j)->jointType() == cnoid::Link::JointType::REVOLUTE_JOINT){
          cnoid::Vector3 dp = omega.cross(target_p - path_.joint(j)->p());
          jacobian_.coeffRef(0,col)=dp[0];
          jacobian_.coeffRef(1,col)=dp[1];
          jacobian_.coeffRef(2,col)=dp[2];
          jacobian_.coeffRef(3,col)=omega[0];
          jacobian_.coeffRef(4,col)=omega[1];
          jacobian_.coeffRef(5,col)=omega[2];
        }
      }

      return jacobian_;
    }

    const Eigen::SparseMatrix<double,Eigen::RowMajor>& ContactPoint::calcRinv(cnoid::Link* parent, cnoid::Position& T_local) {
      Rinv_.resize(6,6);
      cnoid::Matrix3 Rtrans = ( parent->R() * T_local.linear() ).transpose();
      for(size_t j=0;j<3;j++){
        for(size_t k=0;k<3;k++){
          Rinv_.coeffRef(j,k) = Rtrans(j,k);
          Rinv_.coeffRef(3+j,3+k) = Rtrans(j,k);
        }
      }

      return Rinv_;
    }

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
