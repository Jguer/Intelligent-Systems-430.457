#include <cmath>
#include <cstdio>
#include <project2/pid.h>

#define PI 3.14159265358979323846

PID::PID(double Kp_in, double Ki_in, double Kd_in) {
  Kp = Kp_in;
  Ki = Ki_in;
  Kd = Kd_in;

  error = 0;
  error_diff = 0;
  error_sum = 0;

  freq = 0.016666;
}

PID::~PID() {}

float PID::get_control(point car_pose, traj goal_pose) {
  float ctrl;
  float p;
  float i;
  float d;
  float des_angle;

  // Angle
  des_angle =
      atan2(goal_pose.y - car_pose.y, goal_pose.x - car_pose.x) - car_pose.th;
  if (des_angle > PI) {
    des_angle = -2 * PI + des_angle;
  } else if (des_angle < -PI) {
    des_angle = 2 * PI + des_angle; 
  }

  // Updating Error
  error_diff = des_angle - error;
  error = des_angle;
  error_sum += error * freq;

  // Calculating P I D
  p = Kp * error;
  i = Ki * error_sum;
  d = Kd * error_diff / freq;

  // Control Value
  ctrl = p + i + d;

  return ctrl;
}
