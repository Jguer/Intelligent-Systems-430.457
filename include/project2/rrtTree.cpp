#include "rrtTree.h"
#include <algorithm>
#include <cstdlib>
#include <ros/ros.h>
#include <unistd.h>
#define PI 3.14159265358979323846

double max_alpha = 0.2;
double L = 0.325;

rrtTree::rrtTree() {
  count = 0;
  root = NULL;
  ptrTable[0] = NULL;
  countGoalBias = 4;
}

rrtTree::rrtTree(point x_init, point x_goal) {
  this->x_init = x_init;
  this->x_goal = x_goal;

  std::srand(std::time(NULL));
  count = 1;
  root = new node;
  ptrTable[0] = root;
  root->idx = 0;
  root->idx_parent = 0;
  root->location = x_init;
  root->rand = x_init;
  root->alpha = 0;
  root->d = 0;
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
  this->map_original = map.clone();
  this->map = addMargin(map, margin);
  this->map_origin_x = map_origin_x;
  this->map_origin_y = map_origin_y;
  this->res = res;
  srand(time(NULL));

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
      x1 = cv::Point((int)(Res * (p1_y / res + map_origin_y)),
                     (int)(Res * (p1_x / res + map_origin_x)));
      x2 = cv::Point((int)(Res * (p2_y / res + map_origin_y)),
                     (int)(Res * (p2_x / res + map_origin_x)));
      cv::line(imgResult, x1, x2, cv::Scalar(255, 0, 0), thickness, lineType);
    }
  }
  cv::namedWindow("Mapping");
  cv::Rect imgROI((int)Res * 200, (int)Res * 200, (int)Res * 400,
                  (int)Res * 400);
  cv::imshow("Mapping", imgResult(imgROI));
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

  cv::circle(imgResult,
             cv::Point((int)(Res * (path[0].y / res + map_origin_y)),
                       (int)(Res * (path[0].x / res + map_origin_x))),
             radius, cv::Scalar(0, 0, 255), CV_FILLED);
  cv::circle(
      imgResult,
      cv::Point((int)(Res * (path[path.size() - 1].y / res + map_origin_y)),
                (int)(Res * (path[path.size() - 1].x / res + map_origin_x))),
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
  traj x_new;
  bool valid = false;
  int neighbor_id;

  printf("Here 1\n");
  // initialization of x_near and x_new at start
  x_near = this->x_init;

  x_new.x = this->x_init.x;
  x_new.y = this->x_init.y;
  x_new.th = this->x_init.th;
  x_new.alpha = 0;
  x_new.d = 0;

  printf("Here 2\n");
  // building vector x_init to x_goal
  // checking if distance of x_near is close enough to reach in last step
  while (sqrt((pow(x_new.x - this->x_goal.x, 2)) +
              (pow(x_new.y - this->x_goal.y, 2))) > MaxStep) {

    printf("Distance from goal %0.2f\n",
           sqrt((pow(x_new.x - this->x_goal.x, 2)) +
                (pow(x_new.y - this->x_goal.y, 2))));
    // checking if path is free of obstacles
    printf("Here 3\n");
    do {
      x_rand = this->randomState(x_max, x_min, y_max, y_min, x_goal);
      neighbor_id = this->nearestNeighbor(x_rand, MaxStep);
      if (neighbor_id == -1) {
        printf("Here neighbor\n");
        continue;
      }
      x_near = this->ptrTable[neighbor_id]->location;
      printf("Here newState\n");
      valid = this->newState(&x_new, x_near, x_rand, MaxStep);
    } while (valid == false);
    point p_new;
    p_new.x = x_new.x;
    p_new.x = x_new.y;
    p_new.th = x_new.th;
    printf("Here 5\n");
    this->addVertex(p_new, x_rand, neighbor_id, x_new.alpha, x_new.d);
    printf("Here 6\n");

    printf("Pushed %.2f, %.2f, %.2f\n", p_new.x, p_new.y, p_new.th);
    path.push_back(x_new);
  }
  printf("Here 7\n");
  std::reverse(path.begin(), path.end());
  printf("Here 7.1\n");
  x_new = new traj;
  x_new.x = this->x_goal.x;
  x_new.y = this->x_goal.y;
  x_new.th = this->x_goal.th;
  x_new.alpha = 0;
  x_new.d = 0;
  printf("Here 8\n");
  path.push_back(x_new);

  return path;
}

point rrtTree::randomState(double x_max, double x_min, double y_max,
                           double y_min, point x_goal) {
  point x_rand;

  if (rrtTree::countGoalBias == 0) {
    x_rand = x_goal;
    rrtTree::countGoalBias = 4;
  } else {
    x_rand.x = static_cast<double>(rand()) / (x_max - x_min) + x_min;
    x_rand.y = static_cast<double>(rand()) / (y_max - y_min) + y_min;
    x_rand.th = atan2(x_rand.y, x_rand.x);
    --rrtTree::countGoalBias;
  }

  return x_rand;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
  int distance_min;
  int idx_near = -1;

  distance_min = INT_MAX;
  for (int i = 0; i < this->count; i++) {
    point x_near = this->ptrTable[i]->location;
    double dist_to_rand =
        sqrt((pow(x_near.x - x_rand.x, 2)) + (pow(x_near.y - x_rand.y, 2)));

    double max_th = x_near.th + (MaxStep * tan(max_alpha)) / (L);
    if (fabs(x_rand.th) > max_th) {
      continue;
    }

    if (dist_to_rand != 0 && dist_to_rand < distance_min) {
      distance_min = dist_to_rand;
      idx_near = i;
    }
  }
  return idx_near;
}

bool rrtTree::isCollision(point x1, point x2, double d, double R) {
  double x_c = x1.x - R * sin(x1.th);
  double y_c = x1.y + R * cos(x1.th);

  for (double n = 0; n <= d; n += 0.5) {
    double beta = n / R;
    double new_x = x_c + R * sin(x1.th + beta);
    double new_y = y_c - R * cos(x1.th + beta);

    int i = static_cast<int>(round(new_x / this->res + this->map_origin_x));
    int j = static_cast<int>(round(new_y / this->res + this->map_origin_y));

    printf("Checking (%0.2f %0.2f)->(%d %d)(%d)for collision.\n", new_x, new_y,
           i, j, this->map.at<uchar>(i, j));

    if (this->map.at<uchar>(i, j) != 255) {
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
    double dist_to_rand =
        sqrt((pow(x_near.x - x_rand.x, 2)) + (pow(x_near.y - x_rand.y, 2)));

    if (dist_to_rand != 0 && dist_to_rand < distance_min) {
      distance_min = dist_to_rand;
      idx_near = i;
    }
  }
  return idx_near;
}

// Returns:
// true - valid
// false - invalid
bool rrtTree::newState(traj *x_new, point x_near, point x_rand,
                       double MaxStep) {
  traj *tmp_traj = new traj;
  tmp_traj->x = -1;
  tmp_traj->y = -1;
  tmp_traj->th = -1;

  for (int i = 0; i < 10; i++) {
    double alpha =
        -max_alpha +
        static_cast<double>(rand()) /
            (static_cast<double>(RAND_MAX / (max_alpha - (-max_alpha))));

    double R = L / tan(alpha);
    double x_c = x_near.x - R * sin(x_near.th);
    double y_c = x_near.y + R * cos(x_near.th);

    double beta = MaxStep / R;

    double new_x = x_c + R * sin(x_near.th + beta);
    double new_y = y_c - R * cos(x_near.th + beta);
    double new_theta = x_near.th + beta;

    double dist_to_rand =
        sqrt((pow(new_x - x_rand.x, 2)) + (pow(new_y - x_rand.y, 2)));
    if (tmp_traj->x == -1 ||
        dist_to_rand < sqrt((pow(tmp_traj->x - x_rand.x, 2)) +
                            (pow(tmp_traj->y - x_rand.y, 2)))) {
      tmp_traj->x = new_x;
      tmp_traj->y = new_y;
      tmp_traj->th = new_theta;
      tmp_traj->d = MaxStep;
      tmp_traj->alpha = alpha;
    }
  }

  point p_new;
  p_new.x = tmp_traj->x;
  p_new.y = tmp_traj->y;
  p_new.th = tmp_traj->th;
  printf("Point generated %.2f, %.2f, %.2f\n", p_new.x, p_new.y, p_new.th);

  if (this->isCollision(x_near, p_new, tmp_traj->d, L / tan(tmp_traj->alpha))) {
    printf("I'm colliding\n");
    return false;
  }

  return true;
}
