#include "traj.h"

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

traj convertFromPoint(point location, double alpha, double d) {
  traj t_new;
  t_new.x = location.x;
  t_new.y = location.y;
  t_new.th = location.th;
  t_new.alpha = alpha;
  t_new.d = d;
  return t_new;
}
