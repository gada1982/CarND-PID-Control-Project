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
  int cte_count;
  double * cte_values;
  double cte_avg;
  double steer_min;
  double steer_max;
  bool steer_min_max_init;
  
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
  void Init(double Kp, double Ki, double Kd);

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
   * Return the average CTE-Value
   */
  double ReturnCteAvg();
  
  /*
   * Return the maximum CTE-Value
   */
  double ReturnCteMax();
  
  /*
   * Return the minimum steering value
   */
  double ReturnSteerMin();
  
  /*
   * Return the maximum steering value
   */
  double ReturnSteerMax();
  
  // Calculates the avg of all elements of an array
  double avgArray();
  
  // Adds an element to an array at the end of the array
  // and deletes the oldest element
  void addElementToArray(double cte);  
};

#endif /* PID_H */
