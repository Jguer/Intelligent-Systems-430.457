#pragma once
#include <cmath>
#include <cstdio>
#include <cstdlib>

struct point {
  double x;
  double y;
  double th;
  void print();
  double distance(point p2);
  double distance(double x2, double y2);
};

double distance(point p1, point p2);
double distance(point p1, double x, double y);
