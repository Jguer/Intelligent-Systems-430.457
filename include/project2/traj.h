#pragma once
#include "point.h"
#include <cstdio>
#include <cstdlib>

struct traj {
  double x;
  double y;
  double th;
  double d;
  double alpha;
  void print();
  point convertToPoint();
  void set(double new_x, double new_y, double new_th, double new_alpha,
           double new_d);
};

traj convertFromPoint(point location, double alpha, double d);
