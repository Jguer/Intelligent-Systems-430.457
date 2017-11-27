#include "traj.h"

void point::print() { printf("X: %.2f Y:%.2f TH:%.2f\n", x, y, th); }

double point::distance(point p2) {
  return sqrt((pow(x - p2.x, 2)) + (pow(y - p2.y, 2)));
}

void point::set(double x1, double y1) {
  x = x1;
  y = y1;
}

double point::distance(double x2, double y2) {
  return sqrt((pow(x - x2, 2)) + (pow(y - y2, 2)));
}

double distance(point p1, point p2) {
  return sqrt((pow(p1.x - p2.x, 2)) + (pow(p1.y - p2.y, 2)));
}

double distance(point p1, double x, double y) {
  return sqrt((pow(p1.x - x, 2)) + (pow(p1.y - y, 2)));
}

void traj::print() {
  printf("X: %.2f Y:%.2f TH:%.2f D:%.2f A:%.2f\n", x, y, th, d, alpha);
}

point traj::convertToPoint() {
  point p_new;
  p_new.x = x;
  p_new.y = y;
  p_new.th = th;

  return p_new;
}

void traj::set(double new_x, double new_y, double new_th, double new_alpha,
               double new_d) {
  x = new_x;
  y = new_y;
  th = new_th;
  d = new_d;
  alpha = new_alpha;
}

traj convertFromPoint(point location, double alpha, double d) {
  traj t_new;
  t_new.x = location.x;
  t_new.y = location.y;
  t_new.th = location.th;
  t_new.alpha = alpha;
  t_new.d = d;
  return t_new;
}
