#include "PID.h"
#include "json.hpp"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {
  this->init(0.0, 0.0, 0.0);
  is_first_update = true;
}

PID::~PID() = default;

void PID::init(const double &Kp, const double &Ki, const double &Kd) {
  // Initialize the PID coefficients
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  // Reset the errors
  p_error = i_error = d_error = 0.0;
}

void PID::updateError(double cte) {
  double prev_cte;
  if(is_first_update) {
    prev_cte = cte;
    is_first_update = false;
  } else {
    prev_cte = p_error;
  }

  p_error = cte;
  i_error += cte;
  d_error = cte - prev_cte;
}

double PID::totalError() {
  return -Kp * p_error - Ki * i_error - Kd * d_error;
}










































