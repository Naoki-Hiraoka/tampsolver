#include <multicontact_controller/WalkController/WalkControllerROS.h>

int main(int argc,char** argv){
  multicontact_controller::WalkControllerROS worker;
  worker.main(argc,argv);
  return 0;
}
