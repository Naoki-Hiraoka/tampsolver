#ifndef MOTOR_TEMPERATURE_ESTIMATOR_H
#define MOTOR_TEMPERATURE_ESTIMATOR_H

#include <cmath>

class MotorTemperatureEstimator {
public:
  bool isValid(){
    if(K_==0 || Re_<0 || R1_ <= 0 || R2_ <= 0 || Ccoil_ <= 0 || Chousing_ <= 0) return false;
    else return true;
  }

  bool estimate(double tau, double dt){
    if(!isValid()) return false;

    double Qin, Qmid, Qout;
    Qin = Re_ * std::pow(tau/K_, 2);
    Qmid = (Tcoil_ - Thousing_) / R1_;
    Qout = (Thousing_ - Tair_) / R2_;

    Tcoil_ += (Qin - Qmid) / Ccoil_ * dt;
    Thousing_ += (Qmid - Qout) / Chousing_ * dt;
    return true;
  }

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

  double Tcoil() const {return Tcoil_;}
  double& Tcoil() {return Tcoil_;}
  double Thousing() const {return Thousing_;}
  double& Thousing() {return Thousing_;}
  double Tair() const {return Tair_;}
  double& Tair() {return Tair_;}
private:
  double Re_; //Electrical resistance [V/A]
  double K_; //Torque coefficient [Nm/A]
  double Ccoil_; //Heat capacity of coil [J/K]
  double Chousing_;  //Heat capacity of housing [J/K]
  double R1_; //thermal resistance between Coil and Housing [K/W]
  double R2_; //thermal resistance between Housing and Air [K/W]

  double Tcoil_; //Temperature of coil
  double Thousing_; //Temperature of housing
  double Tair_;  //Temperature of air
};

#endif
