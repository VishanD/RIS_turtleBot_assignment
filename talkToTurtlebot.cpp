#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "stdlib.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <cmath>

// global constants
const int RIGHT_EDGE_INDEX = 0;
const int RIGHT_MIDDLE_EDGE_INDEX = 159;
const int FRONT_EDGE_INDEX = 319;
const int LEFT_EDGE_INDEX = 639;
const int LEFT_MIDDLE_EDGE_INDEX = 479;

// const double SIDE_SAFETY_MARGIN = 0.2;
// const double FRONT_SAFETY_MARGIN = 0.8;
const double WALL_FOLLOWING_DIST_MIN = 0.2;
const double WALL_FOLLOWING_DIST_MAX = 0.4;
const double WALL_FOLLOWING_LASER_DIST_MIN = 0.4;
const double WALL_FOLLOWING_LASER_DIST_MAX = 0.8;



// global variables
double front_edge = 0.0;
double right_edge = 0.0;
double right_middle_edge = 0.0;
double left_edge = 0.0;
double left_middle_edge = 0.0;
// double yaw = 0;
double current_x = 0;
double current_y = 0;
bool obstacleAhead = false;


// function prototypes
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double goal_x = 1);
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);

// callback functions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  right_edge = scan->ranges[RIGHT_EDGE_INDEX];
  right_middle_edge = scan->ranges[RIGHT_MIDDLE_EDGE_INDEX];
  front_edge = scan->ranges[FRONT_EDGE_INDEX];
  left_edge = scan->ranges[LEFT_EDGE_INDEX];
  left_middle_edge = scan->ranges[LEFT_MIDDLE_EDGE_INDEX];

  // obstacleAhead = (front_edge < FRONT_SAFETY_MARGIN) ? true : false;
  // ROS_INFO("obstacleAhead : %d", obstacleAhead);

  // double ang = (scan->ranges.size() - scan->ranges.size()/2.0)*scan->angle_increment;
  // ROS_INFO("Distance at max_angle: %f", left_edge);
  // ROS_INFO("Angle when parallel to wall: %f", ang);
}

void ComPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{

  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  // yaw = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
  ros::Subscriber ComPose_sub = n.subscribe<nav_msgs::Odometry>("/odom", 10, ComPoseCallback);

  geometry_msgs::Twist msg;

  ros::Rate loop_rate(100);

  while (ros::ok())
  {
    // TODO : 1. Implement wall following
    // if (!wallOnLeft()) {
    //   turnLeft();
    // }
    // else if (!wallForeward()) {
    //   goStraight();
    // }
    // else {
    //   turnRight();
    // }

    // TODO : 2. Include bumper controllers <-- collision detection
    // TODO : 3. Have function to (basically a feedback controller) check the current heading and location with expected heading and location.
    //        Then apply corrections to stay on expected heading and location. <--error control

    // ROS_INFO("Previous X: %f, Previous Y: %f", current_x, current_y);

    ros::spinOnce();
    goStraight(msg ,loop_rate, twist_pub);
    correctCourse(msg ,loop_rate, twist_pub);
    // ROS_INFO("Current X: %f, Current Y: %f", current_x, current_y);

    // turnRight(msg ,loop_rate, twist_pub);
    // stopRobot(msg ,loop_rate, twist_pub);
    // turnLeft(msg ,loop_rate, twist_pub);
    // stopRobot(msg ,loop_rate, twist_pub);

    loop_rate.sleep();
  }

  return 0;
}

// COMPLETE!
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double goal_x) {
  // double goal_x = 1.0;
  double current_pos_x = 0.0;
  double linear_speed = 0.25;
  ros::WallTime t0 = ros::WallTime::now();

  do {
    ROS_INFO("current_pos_x : %f", current_pos_x);
    msg.linear.x = linear_speed;
    msg.angular.z = 0;
    twist_pub.publish(msg);
    current_pos_x = linear_speed * (ros::WallTime::now().toSec() - t0.toSec());
  } while (current_pos_x < goal_x);

  stopRobot(msg ,loop_rate, twist_pub);
}

// COMPLETE!
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  double angular_speed = (M_PI)/36.0;
  msg.linear.x = 0;
  msg.angular.z =  -angular_speed;
  long double wanted_angle = M_PI/2;
  long double turn_angle = 0.0;
  ros::WallTime t0 = ros::WallTime::now();

  do {
    twist_pub.publish(msg);
    ros::WallTime t1 = ros::WallTime::now();
    turn_angle = wanted_angle - (t1.toSec() - t0.toSec())*angular_speed;
    loop_rate.sleep();

    ROS_INFO("dt %f", t1.toSec() - t0.toSec());
    ROS_INFO("turn_angle %Lf", turn_angle * 180.0 / M_PI);
  } while(turn_angle > 0);

  msg.angular.z =  0;
  twist_pub.publish(msg);
}

// COMPLETE!
void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  double angular_speed = (M_PI)/36.0;
  msg.linear.x = 0;
  msg.angular.z = angular_speed;
  long double wanted_angle = M_PI/2;
  long double turn_angle = 0.0;
  ros::WallTime t0 = ros::WallTime::now();

  do {
    twist_pub.publish(msg);
    ros::WallTime t1 = ros::WallTime::now();
    turn_angle = wanted_angle - (t1.toSec() - t0.toSec())*angular_speed;
    loop_rate.sleep();

    ROS_INFO("dt %f", t1.toSec() - t0.toSec());
    ROS_INFO("turn_angle %Lf", turn_angle * 180.0 / M_PI);
  } while(turn_angle > 0);

  msg.angular.z =  0;
  twist_pub.publish(msg);
}

// COMPLETE!
void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 100; i++) {
    ROS_INFO_STREAM("Robot Stop");
    msg.linear.x = 0;
    msg.angular.z =  0;
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
}

void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  // get distance from the left wall (orientation consider later)
  // if (distance < safe_distance_min)
  //  turnRight()
  //  goStraight(safe_distance_min - distance + 0.2)
  //  turnLeft()
  //  stopRobot()
  // else if (distance > safe_distance_max || NaN) {
  //  turnLeft()
  //  goStraight(distance - safe_distance_max + 0.2)
  //  turnRight()
  //  stopRobot()

  if (left_edge < WALL_FOLLOWING_LASER_DIST_MIN) {
    turnRight(msg ,loop_rate, twist_pub);
    double correction = (WALL_FOLLOWING_DIST_MIN - (left_edge * 0.5)); // 0.5 = Sin 30 deg
    ROS_INFO("correction : %f", correction);
    goStraight(msg ,loop_rate, twist_pub, correction);
    turnLeft(msg ,loop_rate, twist_pub);
    stopRobot(msg ,loop_rate, twist_pub);
  }
  else if (left_edge > WALL_FOLLOWING_LASER_DIST_MAX || std::isnan(left_edge)) {
    turnLeft(msg ,loop_rate, twist_pub);
    double correction = ((left_edge * 0.5) - WALL_FOLLOWING_DIST_MAX); // 0.5 = Sin 30 deg
    ROS_INFO("correction : %f", correction);
    goStraight(msg ,loop_rate, twist_pub, correction);
    turnRight(msg ,loop_rate, twist_pub);
    stopRobot(msg ,loop_rate, twist_pub);
  }
}
