#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
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

  };
};
