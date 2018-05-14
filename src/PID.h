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

  // Must have a value in range of [0, 1)
//  const double CHANGE_PERCENT = 0.1;
  bool is_first_update;
//  double prev_cte, sigma_error, error, _best_error;
//  long _it_count, _it_count_min;
//  unsigned long coeff_index;
//  std::vector<double> _delta_pid;
//  TwiddleSteps _step;
//  int _step, _param_index;
//  // number of steps to allow changes to settle, then to evaluate error
//  int n_settle_steps, n_eval_steps;
//  double total_error;
//  bool tried_adding, tried_subtracting, yes_i_wanna_twiddle;

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

//  void Twiddle(const double &cte);

//  void Twiddle(double cte);

  /**
   * Calculate the total PID error.
   */
  double totalError();
//  double GetValue();

  //void ChangeCoefficient(const unsigned long &index, const double &value);
};

#endif /* PID_H */
