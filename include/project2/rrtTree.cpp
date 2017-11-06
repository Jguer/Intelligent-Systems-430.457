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
  int idx_parent;
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
    idx_parent = this->ptrTable[i]->idx_parent;
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
  double out;
  point new_point = randomState(x_max, x_min, y_max, y_min, this->x_goal);
  int neighbor_id = nearestNeighbor(new_point, MaxStep);
  int new_state = newState(&out, point x_near, point x_rand, MaxStep);
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
    x_rand.th = 0;
    --rrtTree::countGoalBias;
  }

  return x_rand;
}

int rrtTree::nearestNeighbor(point x_rand, double MaxStep) {
  int distance_min;
  int idx_near;

  distance_min = INT_MAX;
  for (int i = 0; i < this->count; i++) {
    point x_near = this->ptrTable[i]->location;
    double dist_to_rand =
        sqrt((pow(x_near.x - x_rand.x, 2)) + (pow(x_near.y - x_rand.y, 2)));

    if (dist_to_rand > MaxStep) {
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
  double delta_x = x1.x - x2.x;
  double delta_y = x1.y - x2.y;

  const float max = std::max(std::fabs(delta_x), std::fabs(delta_y));
  delta_x /= max;
  delta_y /= max;

  for (float n = 0; n < max; ++n) {
    x1.x += delta_x / this->res + this->map_origin_x;
    x1.y += delta_y / this->res + this->map_origin_y;
    if (this->map.at<uchar>(static_cast<int>(round(x1.x)),
                            static_cast<int>(round(x1.y))) != 255) {
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
    double alpha = static_cast<double>(rand()) /
                   (static_cast<double>(RAND_MAX / max_alpha));

    double R = L / tan(alpha);
    double x_c = x_near.x - R * sin(x_near.th);
    double y_c = x_near.y + R * cos(x_near.th);

    double beta = MaxStep / R;

    double new_x = x_c + R * sin(x_near.th + beta);
    double new_y = y_c + R * cos(x_near.th + beta);
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
  p_new.x = tmp_traj->y;
  p_new.th = tmp_traj->th;
  if (this->isCollision(x_near, p_new, tmp_traj->d, L / tan(tmp_traj->alpha))) {
    return false;
  }

  return true;
}
