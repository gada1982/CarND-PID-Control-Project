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
  * Calculate the total PID error -> SteerValue.
  */
  double ReturnSteerValue();
  
  /*
   * Calculate the total P error and modifies it -> TrottleValue.
   */
  double ReturnTrottleValue();
  
  /*
   * TODO
   */
  double ReturnCteMin();
  
  /*
   * TODO
   */
  double ReturnCteMax();
};

#endif /* PID_H */
