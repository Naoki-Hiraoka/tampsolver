#include <multicontact_controller/MotorTemperatureEstimator/MotorTemperatureEstimator.h>

namespace multicontact_controller {
  bool MotorTemperatureEstimator::estimate(double tau, double dt){
    if(!isValid()) return false;

    // estimate coil, housing temperature
    double Qin, Qmid, Qout;
    Qin = Re_ * std::pow(tau/K_, 2);
    Qmid = (Tcoil_ - Thousing_) / R1_;
    Qout = (Thousing_ - Tair_) / R2_;

    Tcoil_ += (Qin - Qmid) / Ccoil_ * dt;
    Thousing_ += (Qmid - Qout) / Chousing_ * dt;

    // estimate maximun effort
    double squareTauMaxSoft =
      (Tlimit_ - Tair_
       - TempPredParams_[1] * (TempPredParams_[2] * Tair_ + TempPredParams_[3] * Tcoil_ + TempPredParams_[4]  * Thousing_) * std::exp(TempPredParams_[6] * tsoft_)
       - TempPredParams_[7] * (TempPredParams_[8] * Tair_ + TempPredParams_[9] * Tcoil_ + TempPredParams_[10]  * Thousing_) * std::exp(TempPredParams_[12] * tsoft_))
      / (TempPredParams_[0]
         + TempPredParams_[1] * TempPredParams_[5] * std::exp(TempPredParams_[6] * tsoft_)
         + TempPredParams_[7] * TempPredParams_[11] * std::exp(TempPredParams_[12] * tsoft_));
    if(squareTauMaxSoft>=0) tauMaxsoft_ = std::sqrt(squareTauMaxSoft);
    else tauMaxsoft_ = 0;

    double squareTauMaxHard =
      (Tlimit_ - Tair_
       - TempPredParams_[1] * (TempPredParams_[2] * Tair_ + TempPredParams_[3] * Tcoil_ + TempPredParams_[4]  * Thousing_) * std::exp(TempPredParams_[6] * thard_)
       - TempPredParams_[7] * (TempPredParams_[8] * Tair_ + TempPredParams_[9] * Tcoil_ + TempPredParams_[10]  * Thousing_) * std::exp(TempPredParams_[12] * thard_))
      / (TempPredParams_[0]
         + TempPredParams_[1] * TempPredParams_[5] * std::exp(TempPredParams_[6] * thard_)
         + TempPredParams_[7] * TempPredParams_[11] * std::exp(TempPredParams_[12] * thard_));
    if(squareTauMaxHard>=0) tauMaxhard_ = std::sqrt(squareTauMaxHard);
    else tauMaxhard_ = 0;

    // estimate balance effort
    double squreTauBalance = (Tcoil_ - Thousing_) / R1_ / Re_ * std::pow(K_,2);
    if(squreTauBalance>=0) tauBalance_ = std::sqrt(squreTauBalance);
    else tauBalance_ = 0;

    // estimate remaining time
    double time;
    for(time=0;time<maxRemainingTime_;time+=remainingTimeStep_){
      double Tcoil_future =
        Tair_ + TempPredParams_[0] * std::pow(tau,2)
        + TempPredParams_[1] * (TempPredParams_[2] * Tair_ + TempPredParams_[3] * Tcoil_ + TempPredParams_[4] * Thousing_ + TempPredParams_[5] * std::pow(tau,2)) * std::exp(TempPredParams_[6] * time)
        + TempPredParams_[7] * (TempPredParams_[8] * Tair_ + TempPredParams_[9] * Tcoil_ + TempPredParams_[10] * Thousing_ + TempPredParams_[11] * std::pow(tau,2)) * std::exp(TempPredParams_[12] * time);
      if (Tcoil_future > Tlimit_) break;
    }
    remainingTime_ = std::min(time,maxRemainingTime_);

    return true;
  }

}
