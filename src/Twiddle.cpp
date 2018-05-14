/*
 * Created by Mohammad.
 */

#include "Twiddle.h"
#include <iostream>
#include <limits>
#include <iomanip>

using namespace std;

Twiddle::Twiddle() {
  _is_initialized = false;
}

Twiddle::~Twiddle() = default;

void Twiddle::increasePotentialChange() {
  _delta_pid[_param_index] *= (1.0 + GROW_RATE);
}

void Twiddle::decreasePotentialChange() {
  _delta_pid[_param_index] *= (1.0 - GROW_RATE);
}

void Twiddle::modifyParameter(double factor) {
  _parameters[_param_index] += factor * _delta_pid[_param_index];
}

void Twiddle::setStep(TwiddleSteps next_step) {
  _step = next_step;
}

void Twiddle::nextIndex() {
  _param_index = (this->_param_index + 1) % this->_delta_pid.size();
}

void Twiddle::onFirstTuningStep(const double &new_error) {
  // Update the best error so far
  setBestError(new_error);

  // Increase the parameter (Kp, Ki, or Kd) by the relevant _delta_pid
  modifyParameter(1.0);

  // Move to the next _step
  setStep(STEP_TWO);
}

void Twiddle::onSuccessToFindBetterSolution(const double &new_error) {
  // Retain the best error so far
  setBestError(new_error);

  // Increase the potential change for the current parameter
  increasePotentialChange();

  // Move to the next parameter's index
  nextIndex();

  // Increase the parameter (Kp, Ki, or Kd) by the relevant _delta_pid
  modifyParameter(1.0);

  // Move to the next _step
  setStep(STEP_TWO);
}

void Twiddle::onFailToFindBetterSolutionByIncreasing() {
  // Reverse back the last change and decrease the parameter (Kp, Ki, or Kd) by the relevant _delta_pid
  modifyParameter(-2.0);

  if (_parameters[_param_index] < 0) {
    _parameters[_param_index] = 0;

    // In this case move to the next parameter's index
    // TODO: Is it really needed?
    nextIndex();
  }

  // Move to the next _step
  setStep(STEP_THREE);
}

void Twiddle::onFailToFineAnyBetterSolution() {
  // Set the current parameter back to its original value
  modifyParameter(1.0);

  // Decrease the potential change for the current parameter
  decreasePotentialChange();

  // Move to the next parameter's index
  nextIndex();

  // Increase the parameter (Kp, Ki, or Kd) by the relevant _delta_pid
  modifyParameter(1.0);

  // Move to the next _step
  setStep(STEP_TWO);
}

void Twiddle::init(const std::vector<double> &parameters, const std::vector<double> &delta_pid) {
  _step = STEP_ONE;
  _best_error = numeric_limits<double>::max();
  _parameters = parameters;
  _delta_pid = delta_pid;

  _is_initialized = true;
}

void Twiddle::tune(PID &controller) {
  if(!_is_initialized) {
    cout << endl << "RunTimeError: The object is not initialized!";
    cout << "To fix the issue, make sure you call \"Twiddle::init()\" before  \"Twiddle::tune()\"" << endl;
    throw;
  }

  const double new_error = controller.totalError();

  switch (_step) {
    case STEP_ONE:
      onFirstTuningStep(new_error);
      break;
    case STEP_TWO:
      if (new_error < _best_error) {
        onSuccessToFindBetterSolution(new_error);
      } else {
        onFailToFindBetterSolutionByIncreasing();
      }
      break;
    case STEP_THREE:
      if (new_error < _best_error) {
        onSuccessToFindBetterSolution(new_error);
      } else {
        onFailToFineAnyBetterSolution();
      }
      break;
    default:
      break;
  }

  // Initialize the controller with the tuned _parameters
  controller.init(_parameters[0], _parameters[1], _parameters[2]);
}

void Twiddle::setBestError(const double &error) {
  _best_error = error;
}

void Twiddle::print() {
  cout << ">> Parameters: [ ";
  for (auto &param : _parameters) {
    cout << left << setw(12) << param << " ";
  }
  cout << " ]" << endl;
}
