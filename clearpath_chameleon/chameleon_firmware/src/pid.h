#ifndef PID_H
#define PID_H

#include "fixed.h"
#include "WProgram.h"


class FixedPIDConstants {
  public:
  const Fixed p;
  const Fixed i;
  const Fixed d;
  const Fixed output_max;
  const Fixed output_nmax;
  
  FixedPIDConstants(Fixed p, Fixed i, Fixed d, Fixed output_max, Fixed output_nmax)
    : p(p), i(i), d(d), output_max(output_max), output_nmax(output_nmax) {};
};


class FixedPID {
  public:
  Fixed input;
  Fixed output;
  Fixed setpoint;
    
  FixedPID(const FixedPIDConstants& params) 
  : input(0), output(0), setpoint(0), params(params) {
    reset();
  }
  
  void reset() {
    integrator = 0;
    last_input = input;
  }
    
  void step() {
    Fixed error = setpoint - input;
    Fixed difference = last_input - input;

    integrator += params.i * error;
    saturate(&integrator);

    output = (params.p * error) + integrator + (params.d * difference);
    d_term = (params.d * difference);
    saturate(&output);
    
    last_input = input;
  }

  void saturate(Fixed* value) {
    if (*value > params.output_max) *value = params.output_max;
    if (*value < params.output_nmax) *value = params.output_nmax;
  }

  private:
  Fixed last_input;
  Fixed integrator;
  Fixed d_term;
 
  const FixedPIDConstants& params;
};


class FixedIncr {
  public:
  Fixed actual;
  Fixed target;
  Fixed rate;  // >0
  Fixed target_max;
  Fixed target_nmax;

  FixedIncr(Fixed rate, Fixed target_max, Fixed target_nmax)
    : rate(rate), target_max(target_max), target_nmax(target_nmax) {
  }

  void step() {
    // Clamp target to allowed range.
    if (target > target_max) {
      target = target_max;
    }
    if (target < target_nmax) {
      target = target_nmax;
    }

    // Move actual toward target at rate.
    if (actual > target) {
      actual -= rate;
      if (actual < target) {
        actual = target;        
      }
    } else if (actual < target) {
      actual += rate;
      if (actual > target) {
        actual = target;        
      }
    }
  }
};

#endif
