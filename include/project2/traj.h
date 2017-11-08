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
  traj convertFromPoint(point location, double alpha, double d);
};
