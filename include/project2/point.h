#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

struct point {
  double x;
  double y;
  double th;
  void point::print();
  double point::distance(point p2);
  double point::distance(double x2, double y2);
};

double distance(point p1, point p2);
double distance(point p1, double x, double y);
