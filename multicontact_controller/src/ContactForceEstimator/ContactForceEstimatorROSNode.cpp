#include <multicontact_controller/ContactForceEstimator/ContactForceEstimatorROS.h>

int main(int argc,char** argv){
  multicontact_controller::ContactForceEstimatorROS worker;
  worker.main(argc,argv);
  return 0;
}
