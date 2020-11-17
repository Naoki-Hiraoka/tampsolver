#ifndef MOTOR_TEMPERATURE_ESTIMATOR_H
#define MOTOR_TEMPERATURE_ESTIMATOR_H

#include <cmath>
#include <vector>

namespace multicontact_controller{
  class MotorTemperatureEstimator {
  public:
    bool isValid(){
      if(K_==0 || Re_<0 || R1_ <= 0 || R2_ <= 0 || Ccoil_ <= 0 || Chousing_ <= 0) return false;
      if(Tlimit_<=Tair_ || tsoft_<=0 || thard_<=0 || TempPredParams_.size()!=13) return false;
      return true;
    }

    bool estimate(double tau, double dt);
    double Re() const {return Re_;}
    double& Re() {return Re_;}
    double K() const {return K_;}
    double& K() {return K_;}
    double Ccoil() const {return Ccoil_;}
    double& Ccoil() {return Ccoil_;}
    double Chousing() const {return Chousing_;}
    double& Chousing() {return Chousing_;}
    double R1() const {return R1_;}
    double& R1() {return R1_;}
    double R2() const {return R2_;}
    double& R2() {return R2_;}

    double Tair() const {return Tair_;}
    double& Tair() {return Tair_;}

    double Tlimit() const {return Tlimit_;}
    double& Tlimit() {return Tlimit_;}
    double tsoft() const {return tsoft_;}
    double& tsoft() {return tsoft_;}
    double thard() const {return thard_;}
    double& thard() {return thard_;}
    std::vector<double> TempPredParams() const {return TempPredParams_;}
    std::vector<double>& TempPredParams() {return TempPredParams_;}

    double maxRemainingTime() const {return maxRemainingTime_;}
    double& maxRemainingTime() {return maxRemainingTime_;}
    double remainingTimeStep() const {return remainingTimeStep_;}
    double& remainingTimeStep() {return remainingTimeStep_;}

    double Tcoil() const {return Tcoil_;}
    double& Tcoil() {return Tcoil_;}
    double Thousing() const {return Thousing_;}
    double& Thousing() {return Thousing_;}
    double tauMaxsoft() const {return tauMaxsoft_;}
    double& tauMaxsoft() {return tauMaxsoft_;}
    double tauMaxhard() const {return tauMaxhard_;}
    double& tauMaxhard() {return tauMaxhard_;}
    double tauBalance() const {return tauBalance_;}
    double& tauBalance() {return tauBalance_;}
    double remainingTime() const {return remainingTime_;}
    double& remainingTime() {return remainingTime_;}

  private:
    double Re_; //Electrical resistance [V/A]
    double K_; //Torque coefficient [Nm/A]
    double Ccoil_; //Heat capacity of coil [J/K]
    double Chousing_;  //Heat capacity of housing [J/K]
    double R1_; //thermal resistance between Coil and Housing [K/W]
    double R2_; //thermal resistance between Housing and Air [K/W]

    double Tair_;  //Temperature of air

    double Tlimit_; //maximum coil temperature[degree]
    double tsoft_; //time constant to calculate maximun_effort_soft [s]
    double thard_; //time constant to calculate maximun_effort_hard [s]
    std::vector<double> TempPredParams_;//temperature prediction params

    double maxRemainingTime_;//[s]
    double remainingTimeStep_;//[s]

    double Tcoil_; //Temperature of coil
    double Thousing_; //Temperature of housing
    double tauMaxsoft_; //maximun_effort_soft[Nm]
    double tauMaxhard_; //maximun_effort_hard[Nm]
    double tauBalance_; //Tcoil not change effort[Nm]
    double remainingTime_; //remining time before Tcoil reaches Tlimit[s]
  };
}

#endif
