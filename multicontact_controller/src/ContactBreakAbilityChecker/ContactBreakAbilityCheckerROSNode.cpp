#include <multicontact_controller/ContactBreakAbilityChecker/ContactBreakAbilityCheckerROS.h>

int main(int argc,char** argv){
  multicontact_controller::ContactBreakAbilityCheckerROS worker;
  worker.main(argc,argv);
  return 0;
}
