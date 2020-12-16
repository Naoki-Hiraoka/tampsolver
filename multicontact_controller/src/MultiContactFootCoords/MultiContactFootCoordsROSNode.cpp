#include <multicontact_controller/MultiContactFootCoords/MultiContactFootCoordsROS.h>

int main(int argc,char** argv){
  multicontact_controller::MultiContactFootCoordsROS worker;
  worker.main(argc,argv);
  return 0;
}
