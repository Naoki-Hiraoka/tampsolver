#include <multicontact_controller/SelfCollisionDetector/SelfCollisionDetectorROS.h>

int main(int argc,char** argv){
  multicontact_controller::SelfCollisionDetectorROS worker;
  worker.main(argc,argv);
  return 0;
}
