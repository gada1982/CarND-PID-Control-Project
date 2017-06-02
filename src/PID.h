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
  double previous_time;
  bool time_init_done;
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
   * Coefficients (steering control)
   */
  double Kp;
  double Ki;
  double Kd;
  
  /*
   * Coefficients (trottle/brake control)
   */
  double Kp_trottle;
  double Kp_offset_trottle;

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
  void Init(double Kp, double Ki, double Kd, double Kp_trottle, double Kp_offset_trottle);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
  * Calculate the total PID error -> SteerValue.
  */
  double ReturnSteerValue();
  
  /*
   * Calculate the total P error and modifies it -> TrottleValue.
   */
  double ReturnTrottleValue();
  
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
