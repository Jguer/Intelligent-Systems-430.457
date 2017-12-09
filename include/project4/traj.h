// Copyright 2017 Dario Wirtz Lucie Bechtet Joao Guerreiro
#ifndef INCLUDE_TRAJ_H_
#define INCLUDE_TRAJ_H_
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
    void set(double x1, double y1);
    bool operator==(point &rhs) const;
};

struct traj : point {
    double d;
    double alpha;
    void print();
    void set(double new_x, double new_y, double new_th, double new_alpha,
             double new_d);
};

double distance(point p1, point p2);
double distance(point p1, double x, double y);
traj convertFromPoint(point location, double alpha, double d);
#endif
