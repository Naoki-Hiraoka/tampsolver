#ifndef MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_TORQUE_JACOBIAN_CALCULATOR_H
#define MULTICONTACT_CONTROLLER_CNOID_BODY_UTILS_TORQUE_JACOBIAN_CALCULATOR_H

#include <cnoid/Body>
#include <cnoid/JointPath>
#include <Eigen/Sparse>

#include <multicontact_controller/lib/CnoidBodyUtils/ContactPoint.h>

namespace multicontact_controller{
  namespace cnoidbodyutils{
    class TorqueJacobianCalculator {
    public:
      TorqueJacobianCalculator(cnoid::Body* robot);

      // いわゆるトルクヤコビアン. [root6dof + numJoints] x [root6dof + numJoints]. world原点がrootの位置にあるとみなして、root6dofは原点まわりの力の釣り合いを考える. そのため、root6dofはrootの位置、world系で、各軸独立(シリアルでない, 位置と回転軸が変位前後で不変).
      // 事前にcalcForWardKinematics(false,false)とcalcCM()が必要
      const Eigen::SparseMatrix<double,Eigen::RowMajor>& calcDg(const cnoid::Vector3 g = cnoid::Vector3(0, 0, 9.80665));
      // 事前にcalcForWardKinematics(false,false)が必要。contactPointsのFを用いる.FはContactPointに固定されており、worldに固定されているのではない
      const Eigen::SparseMatrix<double,Eigen::RowMajor>&  calcDJw(std::vector<std::shared_ptr<ContactPoint> > contactPoints);

      // 最初に一回呼ばれる
      bool generateRelationMap();

      // calcDg中に一回呼ばれる
      // copied from Choreonoid/Body/Jacobian.cpp
      struct SubMass{
        double m;
        cnoid::Vector3 mwc;
      };
      void calcSubMass(cnoid::Link* link);

      enum class Relation { OTHER_PATH, SAME_JOINT, ANCESTOR, DESCENDANT };

    protected:
      // relationMap[i][j]: iがjの何か.
      std::map<cnoid::Link*,std::map<cnoid::Link*, Relation> > relationMap_;
      cnoid::Body* robot_;

      // utility. rootjointの軸
      std::vector<cnoid::Vector3> rootAxis_;

      // cache
      Eigen::SparseMatrix<double,Eigen::RowMajor> Dg_;
      Eigen::SparseMatrix<double,Eigen::RowMajor> DJw_;
      std::vector<SubMass> subMasses_;
    };

  };
};

#endif
