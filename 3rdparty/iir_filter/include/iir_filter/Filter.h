#ifndef IIR_FILTER_FILTER_H
#define IIR_FILTER_FILTER_H
#include <IIRFilter.h>

template <class T> class FirstOrderLowPassFilter2
{
private:
    T prev_value;
    double cutoff_freq, dt, const_param;
public:
    FirstOrderLowPassFilter2 (const double _cutoff_freq, const double _dt, const T init_value) : prev_value(init_value), dt(_dt)
    {
        setCutOffFreq(_cutoff_freq);
    };
    ~FirstOrderLowPassFilter2()
    {
    };
    T passFilter (const T& value)
    {
        prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
        return prev_value;
    };
    T passFilter (const T& value, const double _dt)
    {
      if ( _dt != dt ){
        dt = _dt;
        setCutOffFreq(cutoff_freq);
      }
        prev_value = 1.0/(1+const_param) * prev_value + const_param/(1+const_param) * value;
        return prev_value;
    };
    void reset (const T& value) { prev_value = value; };
    void setCutOffFreq (const double f)
    {
        cutoff_freq = f;
        const_param = 2 * M_PI * cutoff_freq * dt;
    };
    double getCutOffFreq () const { return cutoff_freq; };
    T getCurrentValue () const { return prev_value; };
};


#endif
