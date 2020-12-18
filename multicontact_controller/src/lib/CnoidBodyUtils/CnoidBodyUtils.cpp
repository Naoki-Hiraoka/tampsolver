#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <cnoid/EigenUtil>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/SVD>

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

    size_t calcPseudoInverse(const cnoid::MatrixXd &M, cnoid::MatrixXd &Minv, double sv_ratio){
      {
        Eigen::BDCSVD< cnoid::MatrixXd > svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

        cnoid::VectorX s = svd.singularValues();
        cnoid::MatrixXd U = svd.matrixU();
        cnoid::MatrixXd V = svd.matrixV();

        double threshold=0.0;
        if(s.size() != 0) threshold = s[0] * sv_ratio;//Singular values are always sorted in decreasing order
        cnoid::VectorX sinv(s.size());
        size_t nonzeros = 0;
        for (size_t i=0; i<s.size(); i++){
          if (s[i] < threshold || s[i] == 0.0){
            sinv[i] = 0.0;
          } else {
            sinv[i] = 1.0 / s[i];
            nonzeros++;
          }
        }

        Minv = V * sinv.diagonal() * U.transpose();

        return nonzeros;
      }
    }

  };
};
