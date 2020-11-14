#include <choreonoid_cpp/ChoreonoidCpp.h>

namespace choreonoid_cpp {
  void ChoreonoidCpp::objects(const std::set<cnoid::Body*>& objs){
    if(objects_set_) objects_set_(objs);
  }
  void ChoreonoidCpp::objects(const std::vector<cnoid::Body*>& objs){
    if(objects_vector_) objects_vector_(objs);
  }
  void ChoreonoidCpp::objects(cnoid::Body*& obj){
    if(objects_) objects_(obj);
  }
  void ChoreonoidCpp::drawOn(cnoid::SgNodePtr obj, bool flush){
    if(drawon_) drawon_(obj,flush);
  }
  void ChoreonoidCpp::drawObjects(bool flush){
    if(drawobjects_) drawobjects_(flush);
  }
  void ChoreonoidCpp::flush(){
    if(flush_) flush_();
  }
};
