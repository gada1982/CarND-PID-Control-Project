/*
 * PID.cpp
 *
 * Created on: June 02, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#include "PID.h"
#include <iostream>
#include <math.h>
#include <time.h>

using namespace std;

/*
* PID class
*/

PID::PID() {
  d_error = 0;
  p_error = 0;
  i_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd, double offset) {
  // Init values (steering control)
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  this->offset = offset;
  
  cte_min = 10.0;
  cte_max = 0.0;
}

void PID::UpdateError(double cte) {
  // IMPORTANT: These errors are used for controlling the steering and trottle/brake values
  
  // Set the minimum value for CTE -> Used for analysis
  if(cte < cte_min) {
    cte_min = cte;
  }
  
  // Set the maximum value for CTE -> Used for analysis
  if(cte > cte_max) {
    cte_max = cte;
  }
  
  // Calculate the error values of the PID controller
  // Proportional part
  // (p_error) = difference between a desired setpoint and a measured process variable
  p_error = cte;
  
  // Integral part (i_error):
  // (i_error) = integral between a desired setpoint and a measured process variable over time
  i_error += cte;
  
  // Derivative part (d_error):
  // (d_error) = considers the rate of change of error
  d_error = cte - previous_cte;
  
  // Set previous_cte for the next run of UpdateError()
  previous_cte = cte;
}

double PID::TotalError() {
  double total_error;
  
  // Calculate the new value for the PID controller for the steering value
  total_error = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  // Normalize within [-1, 1]
  if (fabs(total_error)>1.0){
    if (total_error > 0){
      total_error = 1.0;
    } else {
      total_error = -1.0;
    }
  }
  
  return total_error;
}

/*double PID::ReturnTrottleValue() {
  double trottle;
  
  // Calculate the new value for the PID controller for the trottle/brake value
  trottle = (Kp_trottle*(fabs(p_error))) + Kp_offset_trottle;
  
  // Normalize within [-1, 1]
  if (fabs(trottle)>1.0){
    if (trottle > 0){
      trottle = 1.0;
    } else {
      trottle = -1.0;
    }
  }
  return trottle;
}*/

double PID::ReturnCteMin() {
  return cte_min;
}

double PID::ReturnCteMax() {  
  return cte_max;
}



