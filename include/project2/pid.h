#include <cmath>
#ifndef POINT_H
#define POINT_H
#include <project2/point.h>
#include <project2/traj.h>
#endif

class PID{
public:
    PID(double Kp_in, double Ki_in, double Kd_in);
    ~PID();

    //this function makes control output using arguments which are the current value and the target setpoint.
    float get_control(point car_pose, traj goal_pose);

private:

    // you can use this private member variables or additionally define other member variables as you want.
    float error;
    float error_sum;
    float error_diff;
    float Kp;
    float Ki;
    float Kd;
    float freq;
};
