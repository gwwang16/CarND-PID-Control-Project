#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double kp_, double ki_, double kd_) {
	Kp = kp_;
	Ki = ki_;
	Kd = kd_;

	p_error = 0;
	i_error = 0;
	d_error = 0;

  steps = 0;
  total_err = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;

  steps += 1;
}

double PID::TotalError() {

  total_err += (p_error*p_error);

	return total_err;
}

double PID::Controller(){

	return -Kp*p_error - Ki*i_error - Kd*d_error;
}

