#include "point.h"

void point::print() { printf("X: %.2f Y:%.2f TH:%.2f\n", x, y, th); }

double distance(point p1, point p2) {
  return sqrt((pow(p1.x - p2.x, 2)) + (pow(p1.y - p2.y, 2)));
}

double distance(point p1, double x, double y) {
  return sqrt((pow(p1.x - x, 2)) + (pow(p1.y - y, 2)));
}
