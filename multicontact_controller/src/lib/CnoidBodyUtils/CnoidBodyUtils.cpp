#include <multicontact_controller/lib/CnoidBodyUtils/CnoidBodyUtils.h>

#include <cnoid/BodyLoader>
#include <cnoid/SceneGraph>
#include <cnoid/EigenUtil>
#include <cnoid/Jacobian>
#include <ros/package.h>
#include <eigen_conversions/eigen_msg.h>
#include <Eigen/SVD>

namespace multicontact_controller {
  namespace cnoidbodyutils {
    cnoid::Body* loadBodyFromParam(const std::string& paramname){
      ros::NodeHandle n;
      std::string vrml_file;
      if (!n.getParam(paramname, vrml_file)) {
        ROS_WARN("Failed to get param %s",paramname.c_str());
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

    void odomToBody(const nav_msgs::Odometry::ConstPtr& msg, cnoid::Body* robot){
      if(robot){
        // rootのvel, accがない TODO
        // Rootという前提あり TODO
        Eigen::Affine3d pose;
        tf::poseMsgToEigen(msg->pose.pose,pose);
        robot->rootLink()->T() = pose;
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

        Minv = V * sinv.asDiagonal() * U.transpose();

        return nonzeros;
      }
    }

    bool appendRow(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
      if(Ms.size() == 0) {
        Mout.resize(0,Mout.cols());//もとのMoutのcolを用いる
        return true;
      }
      size_t cols = Ms[0].cols();
      size_t rows = 0;
      for(size_t i=0;i<Ms.size();i++){
        rows += Ms[i].rows();
        if(Ms[i].cols() != cols){
          std::cerr << "[appendRow] Ms[i].cols() " << Ms[i].cols() << " != cols " << cols << std::endl;
          return false;
        }
      }
      Mout.resize(rows,cols);
      size_t idx = 0;
      for(size_t i=0;i<Ms.size();i++){
        Mout.middleRows(idx,Ms[i].rows()) = Ms[i];
        idx += Ms[i].rows();
      }

      return true;
    }
    bool appendRow(const std::vector<cnoid::VectorX>& vs, cnoid::VectorX& vout){
      size_t rows = 0;
      for(size_t i=0;i<vs.size();i++){
        rows += vs[i].size();
      }
      vout.resize(rows);
      size_t idx = 0;
      for(size_t i=0;i<vs.size();i++){
        vout.segment(idx,vs[i].size()) = vs[i];
        idx += vs[i].size();
      }

      return true;
    }

    bool appendCol(const std::vector<Eigen::SparseMatrix<double, Eigen::ColMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::ColMajor >& Mout){
      if(Ms.size() == 0) {
        Mout.resize(Mout.rows(),0);//もとのMoutのrowを用いる
        return true;
      }
      size_t rows = Ms[0].rows();
      size_t cols = 0;
      for(size_t i=0;i<Ms.size();i++){
        cols += Ms[i].cols();
        if(Ms[i].rows() != rows){
          std::cerr << "[appendCol] Ms[i].rows() " << Ms[i].rows() << " != rows " << rows << std::endl;
          return false;
        }
      }
      Mout.resize(rows,cols);
      size_t idx = 0;
      for(size_t i=0;i<Ms.size();i++){
        Mout.middleCols(idx,Ms[i].cols()) = Ms[i];
        idx += Ms[i].cols();
      }

      return true;
    }

    bool appendDiag(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Ms, Eigen::SparseMatrix<double, Eigen::RowMajor >& Mout){
      if(Ms.size() == 0) {
        Mout.resize(0,0);
        return true;
      }
      size_t cols = 0;
      size_t rows = 0;
      for(size_t i=0;i<Ms.size();i++){
        rows += Ms[i].rows();
        cols += Ms[i].cols();
      }
      Mout.resize(rows,cols);
      size_t idx_row = 0;
      size_t idx_col = 0;
      for(size_t i=0;i<Ms.size();i++){
        Eigen::SparseMatrix<double, Eigen::ColMajor> M_ColMajor(Ms[i].rows(),cols);
        M_ColMajor.middleCols(idx_col,Ms[i].cols()) = Ms[i];
        Mout.middleRows(idx_row,M_ColMajor.rows()) = M_ColMajor;
        idx_row += Ms[i].rows();
        idx_col += Ms[i].cols();
      }

      return true;
    }

    double dampingFactor(double w,
                         double we,
                         const cnoid::VectorX& b,
                         const cnoid::VectorX& wa,
                         const cnoid::VectorX& dl,
                         const cnoid::VectorX& du,
                         const cnoid::VectorX& wc){
      if(wa.size() != b.size()){
        std::cerr << "[cnoidbodyutils::dampingFactor] wa.size() != b.size()" << std::endl;
        return w;
      }
      if(dl.size() != wc.size() || du.size() != wc.size()){
        std::cerr << "[cnoidbodyutils::dampingFactor] dl.size() != wc.size() || du.size() != wc.size()" << std::endl;
        return w;
      }
      double e = 0;

      Eigen::SparseMatrix<double,Eigen::RowMajor> Wa(wa.size(),wa.size());
      for(size_t i=0;i<wa.size();i++) Wa.insert(i,i) = wa[i];
      e += b.transpose() * Wa * b;

      Eigen::SparseMatrix<double,Eigen::RowMajor> Wc(wc.size(),wc.size());
      for(size_t i=0;i<wc.size();i++) Wc.insert(i,i) = wc[i];
      cnoid::VectorX dl_e = dl;
      for(size_t i=0;i<dl_e.size();i++) if(dl_e[i]<0) dl_e[i] = 0.0;
      cnoid::VectorX du_e = du;
      for(size_t i=0;i<du_e.size();i++) if(du_e[i]>0) du_e[i] = 0.0;
      e += dl_e.transpose() * Wc * dl_e;
      e += du_e.transpose() * Wc * du_e;
      return w + we * e;
    }

    void calcCMJacobian(cnoid::Body* robot, Eigen::SparseMatrix<double, Eigen::RowMajor>& CMJ){
      Eigen::MatrixXd CMJ_dense_flipped;
      cnoid::calcCMJacobian(robot,nullptr,CMJ_dense_flipped); // [joint root]の順
      Eigen::MatrixXd CMJ_dense(CMJ_dense_flipped.rows(),CMJ_dense_flipped.cols()); // [root joint]の順
      CMJ_dense.leftCols(6) = CMJ_dense_flipped.rightCols(6);
      CMJ_dense.rightCols(CMJ_dense.cols()-6) = CMJ_dense_flipped.leftCols(CMJ_dense.cols()-6);
      CMJ.resize(CMJ_dense.rows(),CMJ_dense.cols());
      for(size_t i=0;i<CMJ_dense.rows();i++){
        for(size_t j=0;j<CMJ_dense.cols();j++){
          CMJ.insert(i,j) = CMJ_dense(i,j);
        }
      }
    }

    bool defineMaximumError(const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& As,
                            const std::vector<Eigen::VectorXd>& bs,
                            const std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& Cs,
                            const std::vector<Eigen::VectorXd>& dls,
                            const std::vector<Eigen::VectorXd>& dus,
                            std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& CsHelper,
                            std::vector<Eigen::VectorXd>& dlsHelper,
                            std::vector<Eigen::VectorXd>& dusHelper,
                            std::vector<Eigen::VectorXd>& wcsHelper,
                            std::vector<Eigen::SparseMatrix<double, Eigen::RowMajor> >& C_extsHelper,
                            double& maximum,
                            int As_begin_idx,//Asのいくつ目からを見るか
                            int As_end_idx,//Asのいくつ目までを見るか. 負なら最後まで
                            int Cs_begin_idx,//Csのいくつ目からを見るか
                            int Cs_end_idx//Csのいくつ目までを見るか. 負なら最後まで
                            ){
      if(As_end_idx < 0) As_end_idx = As.size();
      if(Cs_end_idx < 0) Cs_end_idx = Cs.size();

      if(As.size() != bs.size()){
        std::cerr << "[cnoidbodyutils::defineMaximumError] As.size() != bs.size()" << std::endl;
        return false;
      }
      if(As_begin_idx < 0 || As_begin_idx > As.size() || As_end_idx > As.size()){
        std::cerr << "[cnoidbodyutils::defineMaximumError] As_begin_idx < 0 || As_begin_idx > As.size() || As_end_idx > As.size()" << std::endl;
        return false;
      }
      if(Cs.size() != dls.size() || Cs.size() != dus.size()){
        std::cerr << "[cnoidbodyutils::defineMaximumError] Cs.size() != dls.size() || Cs.size() != dus.size()" << std::endl;
        return false;
      }
      if(Cs_begin_idx < 0 || Cs_begin_idx > Cs.size() || Cs_end_idx > Cs.size()){
        std::cerr << "[cnoidbodyutils::defineMaximumError] Cs_begin_idx < 0 || Cs_begin_idx > Cs.size() || Cs_end_idx > Cs.size()" << std::endl;
        return false;
      }

      // CsHelper.clear();
      // dlsHelper.clear();
      // dusHelper.clear();
      // wcsHelper.clear();
      // C_extsHelper.clear();

      // calc maximum error
      maximum = - std::numeric_limits<double>::max();
      for(size_t i=As_begin_idx;i<As_end_idx;i++){
        const cnoid::VectorX& b = bs[i];
        for(size_t j=0;j<b.size();j++){
          double e = std::abs(b[j]);
          if(e > maximum) maximum = e;
        }
      }
      for(size_t i=Cs_begin_idx;i<Cs_end_idx;i++){
        const cnoid::VectorX& du = dus[i];
        for(size_t j=0;j<du.size();j++){
          double e = - du[j];
          if(e > maximum) maximum = e;
        }
        const cnoid::VectorX& dl = dls[i];
        for(size_t j=0;j<dl.size();j++){
          double e = dl[j];
          if(e > maximum) maximum = e;
        }
      }


      // define max variable
      for(size_t i=As_begin_idx;i<As_end_idx;i++){
        {
          const Eigen::SparseMatrix<double,Eigen::RowMajor>& A = As[i];
          const cnoid::VectorX& b = bs[i];
          CsHelper.push_back(A);
          Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(A.rows(),1);
          for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = -1.0;
          C_extsHelper.push_back(tmpC_ext);
          cnoid::VectorX tmpdl(b.size());
          for(size_t j=0;j<tmpdl.size();j++) tmpdl[j] = - std::numeric_limits<double>::max();
          dlsHelper.push_back(tmpdl);
          dusHelper.push_back(b - tmpC_ext * maximum);
          cnoid::VectorX tmpwc(b.size());
          for(size_t j=0;j<tmpwc.size();j++) tmpwc[j] = 1.0;
          wcsHelper.push_back(tmpwc);//解かないので使わない
        }
        {
          const Eigen::SparseMatrix<double,Eigen::RowMajor>& A = As[i];
          const cnoid::VectorX& b = bs[i];
          CsHelper.push_back(A);
          Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(A.rows(),1);
          for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = 1.0;
          C_extsHelper.push_back(tmpC_ext);
          cnoid::VectorX tmpdu(b.size());
          for(size_t j=0;j<tmpdu.size();j++) tmpdu[j] = std::numeric_limits<double>::max();
          dusHelper.push_back(tmpdu);
          dlsHelper.push_back(b - tmpC_ext * maximum);
          cnoid::VectorX tmpwc(b.size());
          for(size_t j=0;j<tmpwc.size();j++) tmpwc[j] = 1.0;
          wcsHelper.push_back(tmpwc);//解かないので使わない
        }
      }
      for(size_t i=Cs_begin_idx;i<Cs_end_idx;i++){
        {
          const Eigen::SparseMatrix<double,Eigen::RowMajor>& C = Cs[i];
          const cnoid::VectorX& du = dus[i];
          CsHelper.push_back(C);
          Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(C.rows(),1);
          for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = -1.0;
          C_extsHelper.push_back(tmpC_ext);
          cnoid::VectorX tmpdl(du.size());
          for(size_t j=0;j<tmpdl.size();j++) tmpdl[j] = -std::numeric_limits<double>::max();
          dlsHelper.push_back(tmpdl);
          dusHelper.push_back(du - tmpC_ext * maximum);
          cnoid::VectorX tmpwc(du.size());
          for(size_t j=0;j<tmpwc.size();j++) tmpwc[j] = 1.0;
          wcsHelper.push_back(tmpwc);//解かないので使わない
        }
        {
          const Eigen::SparseMatrix<double,Eigen::RowMajor>& C = Cs[i];
          const cnoid::VectorX& dl = dls[i];
          CsHelper.push_back(C);
          Eigen::SparseMatrix<double,Eigen::RowMajor> tmpC_ext(C.rows(),1);
          for(size_t j=0;j<tmpC_ext.rows();j++) tmpC_ext.insert(j,0) = 1.0;
          C_extsHelper.push_back(tmpC_ext);
          cnoid::VectorX tmpdu(dl.size());
          for(size_t j=0;j<tmpdu.size();j++) tmpdu[j] = std::numeric_limits<double>::max();
          dusHelper.push_back(tmpdu);
          dlsHelper.push_back(dl - tmpC_ext * maximum);
          cnoid::VectorX tmpwc(dl.size());
          for(size_t j=0;j<tmpwc.size();j++) tmpwc[j] = 1.0;
          wcsHelper.push_back(tmpwc);//解かないので使わない
        }
      }

      return true;
    }

    bool copyBodyKinematicsState(const cnoid::Body* robot_in, cnoid::Body* robot_out){
      if(!robot_in || !robot_out || robot_in->numJoints() != robot_out->numJoints()){
        std::cerr << "[cnoidbodyutils::copyBodyKinematicsState] !robot_in || !robot_out || robot_in->numJoints() != robot_out->numJoints()" << std::endl;
        return false;
      }

      robot_out->rootLink()->T() = robot_in->rootLink()->T();
      for(size_t i=0;i<robot_in->numJoints();i++){
        robot_out->joint(i)->q() = robot_in->joint(i)->q();
      }
      robot_out->calcForwardKinematics(false,false);
    }
  };
};
