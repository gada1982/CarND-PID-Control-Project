/*
 * PID.h
 *
 * Created on: June 02, 2017
 * Author: Daniel Gattringer
 * Mail: daniel@gattringer.biz
 */

#ifndef PID_H
#define PID_H

class PID {
private:
  double previous_cte;
  double cte_min;
  double cte_max;
  
  /*
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;
  
  /*
   * Coefficients 
   */
  double Kp;
  double Ki;
  double Kd;
  double offset;

public:
  
  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd, double offset);

  /*
   * Update the PID error variables given cross track error.
   */
  void UpdateError(double cte);
  
  /*
   * Calculate the total PID error.
   */
  double TotalError();
  
  /*
   * Return the minimum CTE-Value
   */
  double ReturnCteMin();
  
  /*
   * Return the maximum CTE-Value
   */
  double ReturnCteMax();
};

#endif /* PID_H */
