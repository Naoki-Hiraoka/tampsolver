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
        cnoid::Position currentdeviceT = device->link()->T() * device->T_local();
        cnoid::Position realdeviceT;
        realdeviceT.translation() = currentdeviceT.translation();
        Eigen::Quaterniond q;
        tf::quaternionMsgToEigen(msg->orientation,q);
        realdeviceT.linear() = q.normalized().toRotationMatrix();
        cnoid::Position nextrootT = realdeviceT * currentdeviceT.inverse() * robot->rootLink()->T();
        robot->rootLink()->T() = nextrootT;
        robot->calcForwardKinematics(false,false);
      }
    }
  };
};
