#include <project1/pid.h>

PID::PID(double Kp_in, double Ki_in, double Kd_in)  {
    Kp = Kp_in;
    Ki = Ki_in;
    Kd = Kd_in;

    error = 0;
    error_diff = 0;
    error_sum = 0;

    freq = 0.1;
}

PID::~PID() {}

float PID::get_control(point car_pose, point goal_pose) {

    float ctrl;
    float p;
    float i;
    float d;

    // Updating Error
    error_diff = error - (goal_pose.th - car_pose.th);
    error = goal_pose.th - car_pose.th;
    error_sum += error;

    // Calculating P I D
    p = Kp * error;
    i = Ki * freq * error_sum;
    d = Kd * error_diff / freq;
    
    // Control Value
    ctrl = p + i + d;    

    return ctrl;
}
