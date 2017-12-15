#include "rrtTree.h"
#define PI 3.14159265358979323846

double max_alpha = 0.15;
double L = 0.325;

rrtTree::rrtTree() {
    count = 0;
    root = NULL;
    ptrTable[0] = NULL;
}

rrtTree::~rrtTree() {
    for (int i = 0; i < this->count; i++) {
        if (ptrTable[i] != NULL) {
            delete ptrTable[i];
        }
    }
}

rrtTree::rrtTree(std::vector<point> waypoints, cv::Mat map, double map_origin_x,
                 double map_origin_y, double res, int margin) {
    this->waypoints = waypoints;
    this->freeze_id = 0;
    this->x_init = waypoints.at(0);
    this->x_goal = waypoints.at(1);
    this->map_original = map;
    this->map = addMargin(this->map_original, margin);
    this->map_origin_x = map_origin_x;
    this->map_origin_y = map_origin_y;

    this->map_min_x = res * (-map_origin_x);
    this->map_max_x = res * ((2 * map_origin_x + 0.5) - map_origin_x);
    this->map_min_y = res * (-map_origin_y);
    this->map_max_y = res * ((2 * map_origin_y + 0.5) - map_origin_y);
    this->res = res;

    count = 1;
    root = new node;
    ptrTable[0] = root;
    root->idx = 0;
    root->idx_parent = 0;
    root->location = x_init;
    root->rand = x_init;

    // Initialize engine
    this->generator.seed(time(NULL));
}

void rrtTree::visualizeTree() {
    int thickness = 1;
    int lineType = 8;
    double Res = 2;
    cv::Point x1, x2;

    cv::Mat map_c;
    cv::Mat imgResult;
    cv::cvtColor(this->map, map_c, CV_GRAY2BGR);
    cv::resize(map_c, imgResult, cv::Size(), Res, Res);

    for (int i = 1; i < this->count; i++) {
        if (this->ptrTable[i] == NULL) {
            continue;
        }

        int idx_parent = this->ptrTable[i]->idx_parent;
        if (this->ptrTable[idx_parent] == NULL) {
            continue;
        }
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
    cv::waitKey(0);
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
        radius, cv::Scalar(0, 255, 0), CV_FILLED);

    cv::circle(imgResult,
               cv::Point(static_cast<int>(Res * (path[path.size() - 1].y / res +
                                          map_origin_y)),
                         static_cast<int>(Res * (path[path.size() - 1].x / res +
                                          map_origin_x))),
               radius, cv::Scalar(0, 0, 255), CV_FILLED);

    for (int i = 1; i < this->count; i++) {
        if (this->ptrTable[i] == NULL) {
            continue;
        }
        int idx_parent = this->ptrTable[i]->idx_parent;
        if (this->ptrTable[idx_parent] == NULL) {
            continue;
        }
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

    for (auto gs : this->waypoints) {
        cv::circle(imgResult,
                   cv::Point(static_cast<int>(Res * (gs.y / res + map_origin_y)),
                             static_cast<int>(Res * (gs.x / res + map_origin_x))),
                   5, cv::Scalar(0, 88, 139), CV_FILLED);
    }
    cv::namedWindow("Mapping");
    cv::imshow("Mapping", imgResult);
    cv::waitKey(0);
}

void rrtTree::addVertex(point x_new, point x_rand, int idx_near, double alpha,
                        double d) {
    if (this->count == TABLE_SIZE) {
        std::cout << "Table is full. Bump TABLE_SIZE" << std::endl;
        return;
    }
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
    // INIT
    // initialization of x_near and x_new at start
    std::vector<traj> path;
    int x_old_id = 0;
    point x_near;
    traj x_new;
    // building vector x_init to x_goal
    // checking if distance of x_near is close enough to reach in last step
    for (int w = 1; w < waypoints.size(); w++) {
        int x_final_id = 0;
        this->x_goal = waypoints.at(w);
        for (int k = 0; k < K; k++) {
            point x_rand;
            if (k % 10 == 0) {
                x_rand = this->x_goal;
            } else {
                x_rand = this->randomState(x_max, x_min, y_max, y_min);
            }

            /* std::cout << "X_Goal Point: "; */
            /* this->x_goal.print(); */
            /* std::cout << "X_Random Point: "; */
            /* x_rand.print(); */

            int x_near_id = this->nearestNeighbor(x_rand, MaxStep);
            if (x_near_id == -1 || ptrTable[x_near_id] == NULL) {
                continue;
            }

            x_near = ptrTable[x_near_id]->location;

            /* std::cout << "X_Near Point: "; */
            /* x_near.print(); */
            x_new = newState(x_near, x_rand, MaxStep);
            if (x_new.th > 9000) {
                std::cout << "Popin' x_near: ";
                this->ptrTable[x_near_id]->location.print();
                delete this->ptrTable[x_near_id];
                continue;
            }
            /* std::cout << "X_new Point: "; */
            /* x_new.print(); */

            if (this->isCollision(x_near, x_new, MaxStep)) {
                continue;
            }

            /* std::cout << "Added Vertex "; */
            /* x_new.print(); */
            this->addVertex(x_new, x_rand, x_near_id, x_new.alpha, x_new.d);
            if (x_new.distance(x_goal) < 0.3) {
                x_final_id = this->count - 1;
                break;
            }
        }

        if (this->count == 1) {
            std::cout << "Low quality tree" << std::endl;
            path.clear();
            return path;
        }

        if (x_final_id == 0) {
            x_final_id = this->nearestNeighbor(x_goal);
        }

        if (x_final_id == x_old_id) {
            std::cout << "Got stuck" << std::endl;
            path.clear();
            return path;
        }
        x_old_id = x_final_id;
        path.push_back(convertFromPoint(ptrTable[x_final_id]->location,
                                        ptrTable[x_final_id]->alpha,
                                        ptrTable[x_final_id]->d));

        for (int i = x_final_id; i != this->x_init; i = ptrTable[i]->idx_parent) {
            if (ptrTable[i] == NULL) {
                std::cout << "Parent of important node is deleted" << std::endl;
                path.clear();
                return path;
            }
            path.push_back(
                convertFromPoint(ptrTable[ptrTable[i]->idx_parent]->location,
                                 ptrTable[ptrTable[i]->idx_parent]->alpha,
                                 ptrTable[ptrTable[i]->idx_parent]->d));
        }

        this->freeze_id = x_final_id;
        this->x_init = ptrTable[x_final_id]->location;

        printf("Freeze_id: %d Count: %d\n", x_final_id, this->count);
        if (x_final_id != this->count - 1) {
            for (int i = x_final_id + 1; i < this->count; i++) {
                delete ptrTable[i];
            }
            this->count = x_final_id + 1;
        }
        printf("Freeze_id: %d Count: %d\n", x_final_id, this->count);
    }

    this->x_init = waypoints.at(0);
    std::reverse(path.begin(), path.end());

    return path;
}

point rrtTree::randomState(double x_max, double x_min, double y_max,
                           double y_min) {
    point x_rand;
    assert(x_max > x_min && y_max > y_min);

    std::uniform_real_distribution<double> x_dist(x_min, x_max);
    std::uniform_real_distribution<double> y_dist(y_min, y_max);
    x_rand.x = x_dist(generator);
    x_rand.y = y_dist(generator);
    x_rand.th = atan2(x_rand.y, x_rand.x);

    return x_rand;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
    double rel_th;
    int idx_near = -1;

    double distance_min = INT_MAX;
    for (int i = this->freeze_id; i < this->count; i++) {
        if (ptrTable[i] == NULL) {
            continue;
        }
        point x_near = this->ptrTable[i]->location;

        double dist_to_rand = distance(x_near, x_rand);

        double R = L / tan(max_alpha);
        double beta = MaxStep / R;
        double max_th = x_near.th + beta;
        double min_th = x_near.th - beta;

        if (max_th > PI) {
            double temp_th = max_th;
            max_th = min_th;
            min_th = -2 * PI + temp_th;
        } else if (min_th < -PI) {
            double temp_th = min_th;
            min_th = max_th;
            max_th = 2 * PI + temp_th;
        }

        rel_th = atan2((x_rand.y - x_near.y), (x_rand.x - x_near.x));

        if (rel_th >= max_th || rel_th <= min_th) {
            continue;
        }

        if (dist_to_rand < distance_min) {
            distance_min = dist_to_rand;
            idx_near = i;
        }
    }

    return idx_near;
}

bool rrtTree::isCollision(point x_near, traj x_new, double MaxStep) {
    double R = L / tan(x_new.alpha);
    double x_c = x_near.x - R * sin(x_near.th);
    double y_c = x_near.y + R * cos(x_near.th);

    for (double i = 0; i < x_new.d; i += 0.1) {
        double beta = i / R;

        double new_x = x_c + R * sin(x_near.th + beta);
        double new_y = y_c - R * cos(x_near.th + beta);
        if (new_x < this->map_min_x || new_x > this->map_max_x) {
            return true;
        } else if (new_y < this->map_min_y || new_y > this->map_max_y) {
            return true;
        }

        int x_i = round(new_x / res + this->map_origin_x);
        int y_j = round(new_y / res + this->map_origin_y);
        if (0 > x_i || x_i > round(this->map_origin_x * 2 + 0.5)) {
            return true;
        } else if (0 > y_j || y_j > round(this->map_origin_y * 2 + 0.5)) {
            return true;
        } else if (map.at<uchar>(x_i, y_j) < 125) {
            return true;
        }
    }

    return false;
}

int rrtTree::nearestNeighbor(point x_rand) {
    int distance_min;
    int idx_near;

    distance_min = INT_MAX;
    for (int i = this->freeze_id; i < this->count; i++) {
        if (this->ptrTable[i] == NULL) {
            continue;
        }
        point x_near = this->ptrTable[i]->location;
        double dist_to_rand = distance(x_near, x_rand);

        if (dist_to_rand < distance_min) {
            distance_min = dist_to_rand;
            idx_near = i;
        }
    }
    return idx_near;
}

int rrtTree::size() {
    return this->count;
}

// Returns:
// true - valid
// false - invalid
traj rrtTree::newState(point x_near, point x_rand, double MaxStep) {
    double og_dist = INT_MAX;
    traj x_new;
    x_new.set(9000, 9000, 9001, 0, 0);

    std::normal_distribution<double> alpha_dist(0, max_alpha);
    std::uniform_real_distribution<double> d_dist(MaxStep / (MaxStep * 5),
            MaxStep);
    for (int i = 0; i < 50; i++) {
        double alpha = alpha_dist(generator);
        double d = d_dist(generator);

        double R = L / tan(alpha);
        double x_c = x_near.x - R * sin(x_near.th);
        double y_c = x_near.y + R * cos(x_near.th);

        double beta = d / R;

        double new_x = x_c + R * sin(x_near.th + beta);
        double new_y = y_c - R * cos(x_near.th + beta);
        if (new_x < this->map_min_x || new_x > this->map_max_x) {
            continue;
        } else if (new_y < this->map_min_y || new_y > this->map_max_y) {
            continue;
        }

        double new_theta = x_near.th + beta;
        if (new_theta > PI) {
            new_theta = -2 * PI + new_theta;
        } else if (new_theta < -PI) {
            new_theta = 2 * PI + new_theta;
        }

        double dist_to_rand = x_rand.distance(new_x, new_y);
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

cv::Mat addMargin(cv::Mat map, int margin) {
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
