/*
 * Created by Mohammad.
 */

#ifndef PID_TWIDDLE_H
#define PID_TWIDDLE_H

#include "PID.h"
#include <vector>

/**
 * An enum indicating the different steps of the twiddle algorithm
 */
enum TwiddleSteps {
  STEP_ONE,
  STEP_TWO,
  STEP_THREE
};

/**
 * Twiddle is a helper class for tuning the PID controller's coefficients t minimize the average of cross track error.
 */
class Twiddle {
private:
  /**
   * A constant indicating the rate of increasing/decreasing of the potential changes
   */
  const double GROW_RATE = 0.1;

  /**
   * A boolean variable indicating if the twiddle is initialized or not
   */
  bool _is_initialized;

  /**
   * A holder to keep track of tuning algorithm steps
   */
  TwiddleSteps _step;

  /**
   * A variable indicating the best calculated error so far
   */
  double _best_error;

  /**
   * A double vector indicating the parameters of PID controller to minimize the its error.
   * Equivalent to [Kp, Ki, Kd]
   */
  std::vector<double> _parameters;

  /**
   * A double vector indicating the potential changes of @param{_parameters}
   */
  std::vector<double> _delta_pid;

  /**
   * A variable indicating the parameter's index that is modified
   */
  unsigned long _param_index;

  /**
   * A function to increase the potential change for the current parameter
   */
  void increasePotentialChange();

  /**
   * A function to decrease the potential change for the current parameter
   */
  void decreasePotentialChange();

  /**
   * A setter method for the step
   * @param next_step The new step
   */
  void setStep(TwiddleSteps next_step);

  /**
   * A helper method to modify the parameter at index of @param{_param_index} with
   * its relevant potential change (i.e. @param{_delta_pid}[@param{_param_index}] given factor
   * @param factor A double value indicating amount of change
   */
  void modifyParameter(double factor);

  /**
   * A method to move the parameter index to the next one
   */
  void nextIndex();

  /**
   * A helper method for the first step of tuning
   * @param new_error A new value for the controller new error which should be retained
   */
  void onFirstTuningStep(const double &new_error);

  /**
   * A helper method in the case of finding a better solution for the parameters
   * @param new_error A new value for the controller new error which should be retained
   */
  void onSuccessToFindBetterSolution(const double &new_error);

  /**
   * A helper method in the case of failing to find a better solution by increasing the current parameter
   */
  void onFailToFindBetterSolutionByIncreasing();

  /**
   * A helper method in the case of failing to find a better solution by increasing and decreasing
   * the current parameter with its relevant potential change
   */
  void onFailToFineAnyBetterSolution();

public:
  /**
   * Constructor method
   */
  Twiddle();

  /**
   * Destructor method
   */
  ~Twiddle();

  /**
   * Initialize the created Twiddle
   * @param parameters A vector indicating the initial values of the parameters
   * @param delta_pid A vector indicating the initial values of the potential changes
   */
  void init(const std::vector<double> &parameters, const std::vector<double> &delta_pid);

  /**
   * The function is responsible for tuning the controller's coefficients given PID.
   * @param controller A reference to PID controller which its coefficients needs to be tuned
   */
  void tune(PID &controller);

  /**
   * A setter method for the best error so far
   * @param error A new value for the best error
   */
  void setBestError(const double &error);

  /**
   * A method method to print current parameters
   */
  void print();
};

#endif //PID_TWIDDLE_H
