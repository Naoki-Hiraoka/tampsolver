#ifndef ENDEFFECTOR_H
#define ENDEFFECTOR_H

#include <cnoid/Body>
#include <cnoid/SceneDrawables>
#include <yaml-cpp/yaml.h>
#include <Eigen/Sparse>

namespace RobotConfig {

  class Contact {
  public:
    // endeffector座標系，endeffectorまわり lb <= Cx <=ub
    virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& lb, Eigen::VectorXd& ub);
    // endeffector座標系，endeffectorまわり Ax = b, Cx >= d
    virtual void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d);
    virtual std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position());
  protected:

  };

  class SurfaceContact : public Contact {
  public:
    SurfaceContact(cnoid::SgPolygonMeshPtr surface, double mu_trans, double mu_rot, double max_fz, double min_fz);
    // endeffector座標系，endeffectorまわり lb <= Cx <=ub
    void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& lb, Eigen::VectorXd& ub) override;
    // endeffector座標系，endeffectorまわり Ax = b, Cx >= d
    void getContactConstraint(Eigen::SparseMatrix<double,Eigen::RowMajor>& A, Eigen::VectorXd& b, Eigen::SparseMatrix<double,Eigen::RowMajor>& C, Eigen::VectorXd& d) override;
    std::vector<cnoid::SgNodePtr> getDrawOnObjects(const cnoid::Position& T=cnoid::Position()) override;
  private:
    cnoid::SgPolygonMeshPtr surface;//polygonは一つまで．凸形状限定.endeffector相対
    double mu_trans;
    double mu_rot;
    double max_fz;
    double min_fz;

    cnoid::SgLineSetPtr lines;//for visualization
  };

  class GraspContact : public Contact {
  };

  class EndEffector{
  public:
    EndEffector(const std::string& name, cnoid::Link* link, const cnoid::Position& localpos, std::shared_ptr<Contact> contact);

    std::string& getname() {return name;}
    cnoid::Link* getlink() {return link;}
    cnoid::Position& getlocalpos() {return localpos;}
    std::shared_ptr<Contact> getcontact() {return contact;}
  private:
    std::string name;
    cnoid::Link* link;
    cnoid::Position localpos;

    std::shared_ptr<Contact> contact;
  };

  void readEndEffectorFromProperties(std::map<std::string, std::shared_ptr<EndEffector> >& endeffectors, cnoid::Body* robot, const YAML::Node& property);

};

#endif
