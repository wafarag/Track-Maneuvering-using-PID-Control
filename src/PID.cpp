#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {

  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  prev_cte        = 0.0;
  samples_counter = 0;
  Total_SE = 0;

}

void PID::UpdateError(double cte) {

    // Calculate the 3 error values
    this->p_error = cte;
    this->i_error = this->i_error + cte;
    this->d_error = cte - this->prev_cte;

    // set the previous cte value to the current cte for the next iteration
    this->prev_cte = cte;
    // Increment the counter
    this->samples_counter++;
    // Calculate the Total Squared Error
    this->Total_SE = this->Total_SE + cte*cte;

}

double PID::TotalError() {

    #define POS_STEER_LIMIT    1.0
    #define NEG_STEER_LIMIT   -1.0

    // Calculate the controller output based on the calculated error values
    double steer_value = - ((this->Kp * this->p_error) + (this->Ki * this->i_error) + (this->Kd * this->d_error));

    // Check the Constraints (Upper & Lower Limits)
    if ( steer_value >  POS_STEER_LIMIT) steer_value  = POS_STEER_LIMIT;
    if ( steer_value <  NEG_STEER_LIMIT) steer_value  = NEG_STEER_LIMIT;

    return steer_value;
}

