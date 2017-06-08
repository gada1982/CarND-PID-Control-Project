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
  d_error = 0.0;
  p_error = 0.0;
  i_error = 0.0;
  
  cte_count = 100;
  cte_values = new double[cte_count];
}

PID::~PID() {
  delete cte_values;
}

void PID::Init(double Kp, double Ki, double Kd) {
  // Init values (steering control)
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  steer_min_max_init = false;
  
  cte_min = 10.0;
  cte_max = -10.0;
  
  for(int i=0; i<cte_count; i++){
    cte_values[i] = 0.0;
  }
  
  steer_min = 10.0;
  steer_max = -10.0;
}

void PID::UpdateError(double cte) {
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
  
  if(steer_min_max_init == false && cte > -0.5 && cte < 0.5) {
    steer_min_max_init = true;
  }
  
  // Set the minimum value for CTE -> used for analysis
  if(cte < cte_min) {
    cte_min = cte;
  }
  
  // Set the maximum value for CTE -> used for analysis
  if(cte > cte_max) {
    cte_max = cte;
  }

  // Set the average value for CTE -> used for analysis
  addElementToArray(cte);
  cte_avg = avgArray();
}

double PID::TotalError() {
  double steer;
  
  // Calculate the new value for the PID controller for the steering value
  steer = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  // Normalize within [-1, 1]
  if (fabs(steer)>1.0){
    if (steer > 0){
      steer = 1.0;
    } else {
      steer = -1.0;
    }
  }
  
  // Set the minimum steering value -> Used for analysis
  if(steer_min_max_init == true && steer < steer_min) {
    steer_min = steer;
  }
  
  // Set the maximum steering value -> Used for analysis
  if(steer_min_max_init == true && steer > steer_max) {
    steer_max = steer;
  }

  return steer;
}

double PID::ReturnCteMin() {
  return cte_min;
}

double PID::ReturnCteAvg() {
  return cte_avg;
}

double PID::ReturnCteMax() {  
  return cte_max;
}

double PID::ReturnSteerMin() {
  return steer_min;
}

double PID::ReturnSteerMax() {
  return steer_max;
}

double PID::avgArray(){
  double sum = 0.0;
  for(int i=0; i<cte_count; i++){
    sum+=cte_values[i];
  }
  return sum/cte_count;
}

void PID::addElementToArray(double cte) {
  for(int i=0; i<cte_count-1; i++){
    cte_values[i] = cte_values[i+1];
  }
  cte_values[cte_count-1] = cte;
}

