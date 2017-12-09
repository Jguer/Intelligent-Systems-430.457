// Copyright 2017 Dario Wirtz Lucie Bechtet Joao Guerreiro
#ifndef INCLUDE_PROJECT4_RRTTREE_H_
#define INCLUDE_PROJECT4_RRTTREE_H_
#include <algorithm>
#include <climits>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <project4/traj.h>
#include <random>
#include <ros/ros.h>
#include <unistd.h>

#define TABLE_SIZE 10000

class rrtTree {
private:
    struct node {
        int idx;
        point rand;
        point location;
        int idx_parent;
        double alpha;
        double d;
    } * root;

    std::default_random_engine generator;
    std::normal_distribution<double> alpha_dist;
    std::vector<point> waypoints;
    int count;
    int freeze_id;
    point x_init, x_goal;
    cv::Mat map;
    cv::Mat map_original;
    double map_origin_x, map_origin_y, map_min_x, map_min_y, map_max_x, map_max_y,
           res;
    node *ptrTable[TABLE_SIZE];

    void addVertex(point x_new, point x_rand, int idx_near, double alpha,
                   double d);
    int nearestNeighbor(point x_rand, double MaxStep);
    int nearestNeighbor(point x_rand);
    bool isCollision(point x_near, traj x_new, double MaxStep);
    point randomState(double x_max, double x_min, double y_max, double y_min);
    traj newState(point x_near, point x_rand, double MaxStep);

public:
    rrtTree();
    rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x,
            double map_origin_y, double res, int margin);
    ~rrtTree();

    int size();
    void visualizeTree();
    void visualizeTree(std::vector<traj> path);
    std::vector<traj> generateRRT(double x_max, double x_min, double y_max,
                                  double y_min, int K, double MaxStep);
};

cv::Mat addMargin(cv::Mat map, int margin);
#endif
