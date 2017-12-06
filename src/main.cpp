// state definition
#define INIT 0
#define PATH_PLANNING 1
#define RUNNING 2
#define FINISH -1

#include <cmath>
#include <pwd.h>
#include <unistd.h>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <project4/pid.h>
#include <project4/rrtTree.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Time.h>
#include <tf/transform_datatypes.h>

// map spec
cv::Mat map;
double res;
int map_y_range;
int map_x_range;
double map_origin_x;
double map_origin_y;
double world_x_min;
double world_x_max;
double world_y_min;
double world_y_max;

// parameters you should adjust : K, margin, MaxStep
int margin = 5;
int K = 2000;
double MaxStep = 2.0;
int waypoint_margin = 24;

// Vectoring
std::vector<point> waypoints;
std::vector<traj> path_RRT;

// Robot Parameters
point robot_pose;
ackermann_msgs::AckermannDriveStamped cmd;
double speed;
double angle;
double max_speed = 2.00;
double max_turn = 60.0 * M_PI / 180.0;

// FSM state
int state;

// function definition
void setcmdvel(double vel, double deg);
void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs);
void set_waypoints();
void generate_path_RRT();

int main(int argc, char **argv) {
    ros::init(argc, argv, "slam_main");
    ros::NodeHandle n;

    // Initialize topics
    ros::Publisher cmd_vel_pub =
        n.advertise<ackermann_msgs::AckermannDriveStamped>(
            "/vesc/high_level/ackermann_cmd_mux/input/nav_0", 1);

    ros::Subscriber gazebo_pose_sub =
        n.subscribe("/amcl_pose", 100, callback_state);
    std::cout << "Initialized Topics" << std::endl;

    // FSM
    state = INIT;
    bool running = true;
    PID *pid_ctrl = new PID(0.6, 0.3, 0.1);
    ros::Rate control_rate(60);
    int look_ahead_idx = 0;

    while (running) {
        switch (state) {
        case INIT: {
            srand(time(NULL));

            // Load Map
            char *user = getpwuid(getuid())->pw_name;
            cv::Mat map_org =
                cv::imread((std::string("/home/") + std::string(user) +
                            std::string("/catkin_ws/src/project4/src/slam_map.pgm"))
                           .c_str(),
                           CV_LOAD_IMAGE_GRAYSCALE);

            cv::transpose(map_org, map);
            cv::flip(map, map, 1);

            map_y_range = map.cols;
            map_x_range = map.rows;
            map_origin_x = map_x_range / 2.0 - 0.5;
            map_origin_y = map_y_range / 2.0 - 0.5;
            world_x_min = -4.5;
            world_x_max = 4.5;
            world_y_min = -13.5;
            world_y_max = 13.5;
            res = 0.05;
            std::cout << "Loaded Map\n" << std::endl;

            if (!map.data) {
                // Check for invalid input
                printf("Could not open or find the image\n");
                return -1;
            }
            state = PATH_PLANNING;
        }
        break;
        case PATH_PLANNING: {
            // Set Way Points
            set_waypoints();
            printf("Set way points\n");

            // RRT
            generate_path_RRT();
            printf("Generate RRT\n");

            ros::spinOnce();
            ros::Rate(0.33).sleep();
            printf("Initialize ROBOT\n");
            state = RUNNING;
        }
        break;
        case RUNNING: {
            if (path_RRT.size() == 0) {
                printf("Path is empty.\n");
                state = FINISH;
                continue;
            }

            double dist_to_target = robot_pose.distance(path_RRT[look_ahead_idx].x,
                                    path_RRT[look_ahead_idx].y);
            if (dist_to_target <= 0.4) {
                std::cout << "New destination" << std::endl;
                printf("x, y : %.2f, %.2f \n", path_RRT[look_ahead_idx].x,
                       path_RRT[look_ahead_idx].y);
                look_ahead_idx++;
                if (look_ahead_idx == path_RRT.size()) {
                    std::cout << "Circuit Complete" << std::endl;
                    state = FINISH;
                }
            }

            speed =
                2.0 - 1.2 / (1.0 + (robot_pose.distance(path_RRT[look_ahead_idx].x,
                                    path_RRT[look_ahead_idx].y)));
            angle = pid_ctrl->get_control(robot_pose, path_RRT[look_ahead_idx]);

            // Validate Speed
            speed = (speed > max_speed) ? max_speed : speed;
            speed = (speed < -max_speed) ? -max_speed : speed;
            // Validate Angle
            angle = (angle > max_turn) ? max_turn : angle;
            angle = (angle < -max_turn) ? -max_turn : angle;

            setcmdvel(speed, angle);
            cmd_vel_pub.publish(cmd);

            /* printf("Debug Parameters\n"); */
            /* printf("Speed, Angle : %.2f, %.2f \n", speed, angle); */
            /* printf("Car Pose : %.2f,%.2f,%.2f,%.2f,%.2f \n", robot_pose.x, */
            /*        robot_pose.y, robot_pose.th, angle,
             * path_RRT[look_ahead_idx].th); */

            ros::spinOnce();
            control_rate.sleep();
        }
        break;
        case FINISH: {
            setcmdvel(0, 0);
            cmd_vel_pub.publish(cmd);
            running = false;
            ros::spinOnce();
            control_rate.sleep();
        }
        break;
        default:
        { } break;
        }
    }
    return 0;
}

void set_waypoints() {
    ///*
    std::srand(std::time(NULL));
    point waypoint_candid[5];
    waypoint_candid[0].x = -3.5;
    waypoint_candid[0].y = 12.0;

    cv::Mat map_margin = map.clone();
    int jSize = map.cols; // the number of columns
    int iSize = map.rows; // the number of rows

    for (int i = 0; i < iSize; i++) {
        for (int j = 0; j < jSize; j++) {
            if (map.at<uchar>(i, j) < 125) {
                for (int k = i - waypoint_margin; k <= i + waypoint_margin; k++) {
                    for (int l = j - waypoint_margin; l <= j + waypoint_margin; l++) {
                        if (k >= 0 && l >= 0 && k < iSize && l < jSize) {
                            map_margin.at<uchar>(k, l) = 0;
                        }
                    }
                }
            }
        }
    }

    // TODO 2
    // Make your own code to select waypoints.
    // You can randomly sample some points from the map.
    // Also, the car should follow the track in clockwise.

    /*
    quadrants
        ^y
    1   |   0
    ____|____>x
        |
    2   |   3
    */
    // col == i == x
    // row == j == y
    int quad0_i_max = static_cast<int>(round(map_origin_x));
    int quad0_j_max = static_cast<int>(round(map_origin_y));

    int quad1_i_max = static_cast<int>(round(0));
    int quad1_j_max = static_cast<int>(round(map_origin_y));

    int quad2_i_max = static_cast<int>(round(0));
    int quad2_j_max = static_cast<int>(round(0));

    int quad3_i_max = static_cast<int>(round(map_origin_x));
    int quad3_j_max = static_cast<int>(round(0));

    int quadrants[4][2] = {{quad0_i_max, quad0_j_max},
        {quad1_i_max, quad1_j_max},
        {quad2_i_max, quad2_j_max},
        {quad3_i_max, quad3_j_max}
    };
    std::array<int, 3> quadrantSeq;

    // check in which quadrant starting point is => generate sequence
    if (waypoint_candid[0].y >= 0 && waypoint_candid[0].x >= 0) {
        quadrantSeq = {3, 2, 1};
        printf("Quadrant: %d \n", 0);
    } else if (waypoint_candid[0].y >= 0 && waypoint_candid[0].x <= 0) {
        quadrantSeq = {0, 3, 2};
        printf("Quadrant: %d \n", 1);
    } else if (waypoint_candid[0].y <= 0 && waypoint_candid[0].x <= 0) {
        quadrantSeq = {1, 0, 3};
        printf("Quadrant: %d \n", 2);
    } else if (waypoint_candid[0].y <= 0 && waypoint_candid[0].x >= 0) {
        quadrantSeq = {2, 1, 0};
        printf("Quadrant: %d \n", 3);
    } else {
        printf("Quadrant exception, no waypoint creation possible!");
    }

    // find one Waypoint in each quadrant
    bool foundPoint;
    int i_rand, j_rand;

    for (int i = 0; i < quadrantSeq.size(); i++) {
        foundPoint = false;
        while (foundPoint == false) {
            i_rand = (rand() % static_cast<int>(map_origin_x) +
                      quadrants[quadrantSeq[i]][0]); // x
            j_rand = (rand() % static_cast<int>(map_origin_y) +
                      quadrants[quadrantSeq[i]][1]); // y
            // printf("Random (x,y): %.2f, %.2f \n", x_rand, y_rand);

            if ((map_margin.at<uchar>(i_rand, j_rand)) < 200) {
                // std::cout << "Drop the point."
                //          << "Wob wob wob" << std::endl;
                continue;
            } else {
                foundPoint = true;
                waypoint_candid[i + 1].x = res * (i_rand - map_origin_x);
                waypoint_candid[i + 1].y = res * (j_rand - map_origin_y);
                printf("Waypoint found (x,y): %.2f, %.2f \n", waypoint_candid[i + 1].x,
                       waypoint_candid[i + 1].y);
                // optimization of position of point
            }
        }
    }
    waypoint_candid[4].x = waypoint_candid[0].x;
    waypoint_candid[4].y = waypoint_candid[0].y;

    int order[] = {0, 1, 2, 3, 4};
    int order_size = 5;

    for (int i = 0; i < order_size; i++) {
        waypoints.push_back(waypoint_candid[order[i]]);
    }
}

void callback_state(geometry_msgs::PoseWithCovarianceStampedConstPtr msgs) {
    robot_pose.x = msgs->pose.pose.position.x;
    robot_pose.y = msgs->pose.pose.position.y;
    robot_pose.th = tf::getYaw(msgs->pose.pose.orientation);
    printf("x,y : %f,%f \n", robot_pose.x, robot_pose.y);
}

void setcmdvel(double vel, double deg) {
    cmd.drive.speed = vel;
    cmd.drive.steering_angle = deg;
}

void generate_path_RRT() {
    rrtTree tree;
    for (int i = 0; i < waypoints.size() - 1; i++) {
        if (i == 0) {
            std::cout << "Generating between" << std::endl;
            waypoints.at(i).print();
            waypoints.at(i + 1).print();
            tree = rrtTree(waypoints.at(i), waypoints.at(i + 1), map, map_origin_x,
                           map_origin_y, res, margin);
        } else {
            std::cout << "Generating between" << std::endl;
            path_RRT.back().print();
            waypoints.at(i + 1).print();
            tree = rrtTree(path_RRT.back(), waypoints.at(i + 1), map, map_origin_x,
                           map_origin_y, res, margin);
        }
        std::cout << "Generating Path" << std::endl;
        std::vector<traj> path_tmp = tree.generateRRT(
                                         world_x_max, world_x_min, world_y_max, world_y_min, K, MaxStep);
        printf("New rrtTree generated. Size of Tree: %d\n", tree.size());
        printf("New trajectory generated. Size of Path %zu\n", path_tmp.size());
        tree.visualizeTree(path_tmp);

        for (auto tmp : path_tmp) {
            path_RRT.push_back(tmp);
            tmp.print();
        }
    }
}
