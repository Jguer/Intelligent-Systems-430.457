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
