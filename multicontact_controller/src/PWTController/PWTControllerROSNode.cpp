#include <multicontact_controller/PWTController/PWTControllerROS.h>

int main(int argc,char** argv){
  multicontact_controller::PWTControllerROS worker;
  worker.main(argc,argv);
  return 0;
}
