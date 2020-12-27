#include <multicontact_controller/PCLCollisionDetector/PCLCollisionDetectorROS.h>

int main(int argc,char** argv){
  multicontact_controller::PCLCollisionDetectorROS worker;
  worker.main(argc,argv);
  return 0;
}
