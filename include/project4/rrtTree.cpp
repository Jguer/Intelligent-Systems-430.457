#include "rrtTree.h"
#include <algorithm>
#include <cstdlib>
#include <ros/ros.h>
#include <unistd.h>
#define PI 3.14159265358979323846

double max_alpha = 0.15;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::~rrtTree() {
    for (int i = 1; i <= count; i++) {
        delete ptrTable[i];
    }
}

rrtTree::rrtTree(point x_init, point x_goal, cv::Mat map, double map_origin_x,
                 double map_origin_y, double res, int margin) {
    this->x_init = x_init;
    this->x_goal = x_goal;
    this->map_original = map;
    this->map = addMargin(map, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = 0;
    root->location = x_init;
    root->rand = x_init;
}

cv::Mat rrtTree::addMargin(cv::Mat map, int margin) {
    cv::Mat map_margin = map.clone();
    int xSize = map.cols;
    int ySize = map.rows;

    for (int i = 0; i < ySize; i++) {
        for (int j = 0; j < xSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - margin; k <= i + margin; k++) {
                    for (int l = j - margin; l <= j + margin; l++) {
                        if (k >= 0 && l >= 0 && k < ySize && l < xSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    return map_margin;
}

void rrtTree::visualizeTree() {
    int thickness = 1;
    int lineType = 8;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    for (int i = 1; i < this->count; i++) {
        int idx_parent = this->ptrTable[i]->idx_parent;
        for (int j = 0; j < 10; j++) {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th =
                this->ptrTable[idx_parent]->location.th + d * j / 10 * tan(alpha) / L;
            double p2_th = this->ptrTable[idx_parent]->location.th +
                           d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x = this->ptrTable[idx_parent]->location.x +
                          L / tan(alpha) *
                          (sin(p1_th) - sin(ptrTable[idx_parent]->location.th));
            double p1_y = this->ptrTable[idx_parent]->location.y +
                          L / tan(alpha) *
                          (cos(ptrTable[idx_parent]->location.th) - cos(p1_th));
            double p2_x = this->ptrTable[idx_parent]->location.x +
                          L / tan(alpha) *
                          (sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y +
                          L / tan(alpha) *
                          (cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point(static_cast<int>(Res * (p1_y / res + map_origin_y)),
                           static_cast<int>(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point(static_cast<int>(Res * (p2_y / res + map_origin_y)),
                           static_cast<int>(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(1);
}

void rrtTree::visualizeTree(std::vector<traj> path) {
    int thickness = 1;
    int lineType = 8;
    double Res = 2;
    double radius = 6;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    cv::circle(
        imgResult,
        cv::Point(static_cast<int>(Res * (path[0].y / res + map_origin_y)),
                  static_cast<int>(Res * (path[0].x / res + map_origin_x))),
        radius, cv::Scalar(0, 0, 255), CV_FILLED);
    cv::circle(imgResult,
               cv::Point(static_cast<int>(Res * (path[path.size() - 1].y / res +
                                          map_origin_y)),
                         static_cast<int>(Res * (path[path.size() - 1].x / res +
                                          map_origin_x))),
               radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for (int i = 1; i < this->count; i++) {
        int idx_parent = this->ptrTable[i]->idx_parent;
        for (int j = 0; j < 10; j++) {
            double alpha = this->ptrTable[i]->alpha;
            double d = this->ptrTable[i]->d;
            double p1_th =
                this->ptrTable[idx_parent]->location.th + d * j / 10 * tan(alpha) / L;
            double p2_th = this->ptrTable[idx_parent]->location.th +
                           d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x = this->ptrTable[idx_parent]->location.x +
                          L / tan(alpha) *
                          (sin(p1_th) - sin(ptrTable[idx_parent]->location.th));

            double p1_y = this->ptrTable[idx_parent]->location.y +
                          L / tan(alpha) *
                          (cos(ptrTable[idx_parent]->location.th) - cos(p1_th));

            double p2_x = this->ptrTable[idx_parent]->location.x +
                          L / tan(alpha) *
                          (sin(p2_th) - sin(ptrTable[idx_parent]->location.th));
            double p2_y = this->ptrTable[idx_parent]->location.y +
                          L / tan(alpha) *
                          (cos(ptrTable[idx_parent]->location.th) - cos(p2_th));
            x1 = cv::Point(static_cast<int>(Res * (p1_y / res + map_origin_y)),
                           static_cast<int>(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point(static_cast<int>(Res * (p2_y / res + map_origin_y)),
                           static_cast<int>(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }

    thickness = 3;
    for (int i = 1; i < path.size(); i++) {
        for (int j = 0; j < 10; j++) {
            double alpha = path[i].alpha;
            double d = path[i].d;
            double p1_th =
                path[i - 1].th + d * j / 10 * tan(alpha) / L; // R = L/tan(alpha)
            double p2_th = path[i - 1].th + d * (j + 1) / 10 * tan(alpha) / L;
            double p1_x =
                path[i - 1].x + L / tan(alpha) * (sin(p1_th) - sin(path[i - 1].th));
            double p1_y =
                path[i - 1].y + L / tan(alpha) * (cos(path[i - 1].th) - cos(p1_th));
            double p2_x =
                path[i - 1].x + L / tan(alpha) * (sin(p2_th) - sin(path[i - 1].th));
            double p2_y =
                path[i - 1].y + L / tan(alpha) * (cos(path[i - 1].th) - cos(p2_th));
            x1 = cv::Point(static_cast<int>(Res * (p1_y / res + map_origin_y)),
                           static_cast<int>(Res * (p1_x / res + map_origin_x)));
            x2 = cv::Point(static_cast<int>(Res * (p2_y / res + map_origin_y)),
                           static_cast<int>(Res * (p2_x / res + map_origin_x)));
            cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
        }
    }
    cv::namedWindow("Mapping");
    cv::Rect imgROI(static_cast<int>(Res * 200), static_cast<int>(Res * 200),
                    static_cast<int>(Res * 400), static_cast<int>(Res * 400));
    cv::imshow("Mapping", imgResult(imgROI));
    cv::waitKey(1);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha,
                        double d) {
    node *new_node = new node;
    new_node->idx = this->count;
    new_node->idx_parent = idx_near;
    new_node->location = x_new;
    new_node->rand = x_rand;
    new_node->alpha = alpha;
    new_node->d = d;

    ptrTable[this->count] = new_node;
    this->count++;

    return;
}

std::vector<traj> rrtTree::generateRRT(double x_max, double x_min, double y_max,
                                       double y_min, int K, double MaxStep) {
    std::vector<traj> path;
    point x_rand;
    point x_near;
    int x_near_id;
    traj x_new;

    // INIT
    // initialization of x_near and x_new at start
    x_near = x_init;
    x_new = convertFromPoint(x_init, 0, 0);

    // building vector x_init to x_goal
    // checking if distance of x_near is close enough to reach in last step
    for (int k = 0; k < K; k++) {
        x_rand = this->randomState(x_max, x_min, y_max, y_min);
        if (k % 10 == 0) {
            x_rand = this->x_goal;
        }

        x_near_id = this->nearestNeighbor(x_rand, MaxStep);
        if (x_near_id == -1) {
            continue;
        }

        x_near = ptrTable[x_near_id]->location;

        std::cout << "X_Near Point: ";
        x_near.print();
        x_new = newState(x_near, x_rand, MaxStep);
        std::cout << "X_new Point: ";
        x_new.print();

        if (this->isCollision(x_near, x_new, MaxStep, L / tan(x_new.alpha))) {
            continue;
        }

        /* std::cout << "Added Vertex "; */
        /* x_new.print(); */
        this->addVertex(x_new, x_rand, x_near_id, x_new.alpha, x_new.d);
    }

    x_near_id = this->nearestNeighbor(x_goal, MaxStep);
    for (int i = x_near_id; i != 0; i = ptrTable[i]->idx_parent) {
        path.push_back(convertFromPoint(ptrTable[i]->location, ptrTable[i]->alpha,
                                        ptrTable[i]->d));
    }

    std::reverse(path.begin(), path.end());

    return path;
}

point rrtTree::randomState(double x_max, double x_min, double y_max,
                           double y_min) {
    point x_rand;

    x_rand.x = rand() / (double)RAND_MAX * (x_max - x_min) + x_min;
    x_rand.y = rand() / (double)RAND_MAX * (y_max - y_min) + y_min;
    x_rand.th = atan2(x_rand.y, x_rand.x);

    return x_rand;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    double distance_min, dist_to_rand, R, beta, max_th, new_x, new_y, min_th,
           temp_th;
    double rel_th;
    int idx_near = -1;
    point x_near;

    distance_min = INT_MAX;
    for (int i = 0; i < this->count; i++) {
        x_near = this->ptrTable[i]->location;
        dist_to_rand = distance(x_near, x_rand);

        R = L / tan(max_alpha);
        beta = MaxStep / R;
        max_th = x_near.th + beta;
        min_th = x_near.th - beta;

        if (max_th > PI) {
            temp_th = max_th;
            max_th = min_th;
            min_th = -2 * PI + temp_th;
        } else if (min_th < -PI) {
            temp_th = min_th;
            min_th = max_th;
            max_th = 2 * PI + temp_th;
        }

        rel_th = atan2((x_rand.y - x_near.y), (x_rand.x - x_near.x));

        if (rel_th >= max_th || rel_th <= min_th) {
            /*
            std::cout << "Fell out (" << rel_th << "). Limits were: (" << max_th
                      << "," << min_th << "," << x_near.th << "," << beta << ") Point:
            "; x_near.print();
            */
            continue;
        }

        if (dist_to_rand < distance_min) {
            distance_min = dist_to_rand;
            idx_near = i;
        }
    }
    return idx_near;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
    int i;
    printf("X1: %0.2f %0.2f, X2: %0.2f %0.2f", x1.x, x1.y, x2.x, x2.y);
    for (i = 0; i < 100; i++) {
        double x = x1.x + (x2.x - x1.x) * i / 99;
        double y = x1.y + (x2.y - x1.y) * i / 99;
        printf("Map Stuff: %0.2f %0.2f Coords: %0.2f,%0.2f Map: %0.2f,%0.2f \n",
               this->map_origin_x, this->map_origin_y, x, y,
               round(x / res + this->map_origin_x),
               round(y / res + this->map_origin_y));
        if (map.at<uchar>(round(x / res + this->map_origin_x),
                          round(y / res + this->map_origin_y)) != 255) {
            // There was a obstruction
            return true;
        }
    }

    return false;
}

int rrtTree::nearestNeighbor(point x_rand) {
    int distance_min;
    int idx_near;

    distance_min = INT_MAX;
    for (int i = 0; i < this->count; i++) {
        point x_near = this->ptrTable[i]->location;
        double dist_to_rand = distance(x_near, x_rand);

        if (dist_to_rand < distance_min) {
            distance_min = dist_to_rand;
            idx_near = i;
        }
    }
    return idx_near;
}

// Returns:
// true - valid
// false - invalid
traj rrtTree::newState(point x_near, point x_rand, double MaxStep) {
    double og_dist, d, x_c, y_c, alpha, R, beta, new_x, new_y, new_theta,
           dist_to_rand;
    og_dist = INT_MAX;
    traj x_new;

    for (int i = 0; i < 25; i++) {
        alpha = -max_alpha +
                static_cast<double>(rand()) /
                (static_cast<double>(RAND_MAX / (max_alpha - (-max_alpha))));

        d = (MaxStep / 5) +
            static_cast<double>(rand()) /
            (static_cast<double>(RAND_MAX / (MaxStep - (MaxStep / 5))));

        R = L / tan(alpha);
        x_c = x_near.x - R * sin(x_near.th);
        y_c = x_near.y + R * cos(x_near.th);

        beta = d / R;

        new_x = x_c + R * sin(x_near.th + beta);
        new_y = y_c - R * cos(x_near.th + beta);

        new_theta = x_near.th + beta;
        if (new_theta > PI) {
            new_theta = -2 * PI + new_theta;
        } else if (new_theta < -PI) {
            new_theta = 2 * PI + new_theta;
        }

        dist_to_rand = distance(x_rand, new_x, new_y);
        if (dist_to_rand < og_dist) {
            /* printf("Point candidate %.2f, %.2f, %.2f\n", new_x, new_y,
             * new_theta);
             */
            og_dist = dist_to_rand;
            x_new.set(new_x, new_y, new_theta, alpha, d);
        }
    }

    return x_new;
}
