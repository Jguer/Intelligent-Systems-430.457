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
  countGoalBias = 4;

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
  traj x_new;
  bool valid = false;
  int neighbor_id;
  // INIT
  // initialization of x_near and x_new at start
  x_near = this->x_init;

  x_new.x = this->x_init.x;
  x_new.y = this->x_init.y;
  x_new.th = this->x_init.th;
  x_new.alpha = 0;
  x_new.d = 0;

  // building vector x_init to x_goal
  // checking if distance of x_near is close enough to reach in last step

  for (int k = 0; k < K; k++) {
    x_rand = this->randomState(x_max, x_min, y_max, y_min, x_goal);
    neighbor_id = this->nearestNeighbor(x_rand, MaxStep);
    if (neighbor_id == -1) {
      continue;
    }
    x_near = this->ptrTable[neighbor_id]->location;

    valid = this->newState(&x_new, x_near, x_rand, MaxStep);
    if (!valid) {
      continue;
    }

    this->addVertex(x_new.convertToPoint(), x_rand, neighbor_id, x_new.alpha,
                    x_new.d);

    std::cout << "Added Vertex ";
    x_new.print();
  }

  x_rand = x_goal;
  neighbor_id = this->nearestNeighbor(x_rand);
  path.push_back(convertFromPoint(x_goal, 0, 0));
  while (int i = neighbor_id; i != 0; i = ptrTable[i]->idx_parent) {
    path.push_back(convertFromPoint(ptrTable[i].location, alpha, d));
  }
  std::reverse(path.begin(), path.end());

  return path;
}

point rrtTree::randomState(double x_max, double x_min, double y_max,
                           double y_min, point x_goal) {
  point x_rand;

  if (this->countGoalBias == 0) {
    x_rand = x_goal;
    this->countGoalBias = 4;
  } else {
    x_rand.x = x_min + static_cast<double>(rand()) / (x_max - x_min);
    x_rand.y = y_min + static_cast<double>(rand()) / (y_max - y_min);
    x_rand.th = atan2(x_rand.y, x_rand.x);
    this->countGoalBias--;
  }

  return x_rand;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
  double distance_min;
  int idx_near = -1;

  distance_min = INT_MAX;
  for (int i = 0; i < this->count; i++) {
    point x_near = this->ptrTable[i]->location;
    double dist_to_rand = distance(x_near, x_rand);

    double R = L / tan(max_alpha);
    double beta = MaxStep / R;
    double max_theta = x_near.th + beta;

    if (fabs(x_rand.th) >= max_th) {
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
  double x_c = x1.x - R * sin(x1.th);
  double y_c = x1.y + R * cos(x1.th);

  for (double n = 0; n < d; n += 0.4) {
    double beta = n / R;
    double new_x = x_c + R * sin(x1.th + beta);
    double new_y = y_c - R * cos(x1.th + beta);

    int i = static_cast<int>(round(new_x / this->res + this->map_origin_x));
    int j = static_cast<int>(round(new_y / this->res + this->map_origin_y));

    /* printf("Checking (%0.2f %0.2f)->(%d %d)(%d)for collision.\n", new_x,
     * new_y, */
    /*        i, j, this->map.at<uchar>(i, j)); */

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
    double dist_to_rand = distance(x_near, x_rand);

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
  bool changed = false;
  double og_dist = 10000.0;

  for (int i = 0; i < 10; i++) {
    double alpha =
        -max_alpha +
        static_cast<double>(rand()) /
            (static_cast<double>(RAND_MAX / (max_alpha - (-max_alpha))));
    double d =
        static_cast<double>(rand()) / (static_cast<double>(RAND_MAX / MaxStep));

    double R = L / tan(alpha);
    double x_c = x_near.x - R * sin(x_near.th);
    double y_c = x_near.y + R * cos(x_near.th);

    double beta = d / R;

    double new_x = x_c + R * sin(x_near.th + beta);
    double new_y = y_c - R * cos(x_near.th + beta);
    double new_theta = x_near.th + beta;

    double dist_to_rand = distance(x_rand, new_x, new_y);
    if (dist_to_rand < og_dist) {
      /* printf("Point candidate %.2f, %.2f, %.2f\n", new_x, new_y, new_theta);
       */
      changed = true;
      og_dist = dist_to_rand;
      x_new->x = new_x;
      x_new->y = new_y;
      x_new->th = new_theta;
      x_new->d = d;
      x_new->alpha = alpha;
    }
  }
  /* printf("Point generated %.2f, %.2f, %.2f\n", p_new.x, p_new.y, p_new.th);
   */

  if (changed) {
    if (this->isCollision(x_near, x_new->convertToPoint(), x_new->d,
                          L / tan(x_new->alpha))) {
      /* printf("I'm colliding\n"); */
      return false;
    }
  } else {
    return false;
  }

  return true;
}
