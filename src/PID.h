#ifndef PID_H
#define PID_H

//#include <vector>

//enum TwiddleSteps {
//  STEP_ONE,
//  STEP_TWO,
//  STEP_THREE
//};

class PID {
  /**
   * Twiddle variables
   */
  bool is_first_update;

public:
  /**
   * Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * Coefficients
   */
  double Kp;
  double Ki;
  double Kd;

  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   */
  void init(const double &Kp, const double &Ki, const double &Kd);

  /**
   * Update the PID error variables given cross track error.
   */
  void updateError(double cte);

  /**
   * Calculate the total PID error.
   */
  double totalError();
};

#endif /* PID_H */
