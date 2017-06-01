#include "PID.h"
#include <iostream>
#include <math.h>
#include <time.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  d_error = 0;
  p_error = 0;
  i_error = 0;
}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  time_init_done = false;
  cte_min = 10.0;
  cte_max = 0.0;
}

void PID::UpdateError(double cte) {
  double dt;
  double current_time = clock();
  
  if(time_init_done) {
    dt = (current_time - previous_time)/CLOCKS_PER_SEC;
  }
  else {
    dt = 0.001;
    time_init_done = true;
    previous_cte = cte;
  }
  
  previous_time = current_time;
  
  cout << "\ndt: " << dt << endl;
  
  if(cte < cte_min) {
    cte_min = cte;
  }
  
  if(cte > cte_max) {
    cte_max = cte;
  }
  
  p_error = cte;
  i_error += fmin(cte, previous_cte)*dt + ((fabs(cte - previous_cte))*dt)/2;
  d_error = (cte - previous_cte) / dt;
  
  previous_cte = cte;
}

double PID::ReturnSteerValue() {
  double steer;
  
  steer = -Kp*p_error - Kd*d_error - Ki*i_error;
  
  if (fabs(steer)>1.0){
    if (steer > 0){
      steer = 1.0;
    } else {
      steer = -1.0;
    }
  }
  
  return steer;
}

double PID::ReturnTrottleValue() {
  double trottle;
  
  trottle = ((1/fabs(fmax(p_error, 0.001))) ) - 0.15 ;
  
  if (fabs(trottle)>1.0){
    if (trottle > 0){
      trottle = 1.0;
    } else {
      trottle = -1.0;
    }
  }
  return trottle;
}

double PID::ReturnCteMin() {
  
  return cte_min;
}

double PID::ReturnCteMax() {
  
  return cte_max;
}



