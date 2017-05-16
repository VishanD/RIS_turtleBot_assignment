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

const double SIDE_SAFETY_MARGIN = 0.2;
const double FRONT_SAFETY_MARGIN = 0.8;
const double WALL_FOLLOWING_DIST = 1;
const double WALL_DETECTION_DIST = 3.7;


// global variables
double front_edge = 1;
double right_edge = 0.5;
double right_middle_edge = 0.5;
double left_edge = 0.5;
double left_middle_edge = 0.5;
double yaw = 0;
bool obstacleAhead = false;


// function prototypes
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw);
void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
// void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw);
// void initializeRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);


// callback functions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  right_edge = scan->ranges[RIGHT_EDGE_INDEX];
  right_middle_edge = scan->ranges[RIGHT_MIDDLE_EDGE_INDEX];
  front_edge = scan->ranges[FRONT_EDGE_INDEX];
  left_edge = scan->ranges[LEFT_EDGE_INDEX];
  left_middle_edge = scan->ranges[LEFT_MIDDLE_EDGE_INDEX];

  obstacleAhead = (front_edge < FRONT_SAFETY_MARGIN) ? true : false;
  ROS_INFO("obstacleAhead : %d", obstacleAhead);

  double ang = (scan->ranges.size() - scan->ranges.size()/2.0)*scan->angle_increment;
  // ROS_INFO("Distance at max_angle: %f", left_edge);
  // ROS_INFO("Angle when parallel to wall: %f", ang);
}

void ComPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  yaw = tf::getYaw(msg->pose.pose.orientation);
  ROS_INFO("yaw %f", yaw);
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

  // TODO : Include bumper controllers
  while (ros::ok())
  {
    // if (!wallOnLeft()) {
    //   turnLeft();
    // }
    // else if (!wallForeward()) {
    //   goStraight();
    // }
    // else {
    //   turnRight();
    // }

    ROS_INFO("yaw %f", yaw);
    turnRight(msg ,loop_rate, twist_pub, yaw);
    stopRobot(msg ,loop_rate, twist_pub);
    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 10; i++) {
    ROS_INFO_STREAM("Go straight");
    if (obstacleAhead) {
      msg.linear.x = 0.0;
      msg.angular.z = 0;
      ROS_INFO_STREAM("Stopping");
    }
    else {
      msg.linear.x = 0.2;
      msg.angular.z = 0;
    }
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
}

void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw) {
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

void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 10; i++) {
    ROS_INFO_STREAM("Turn left");
    msg.linear.x = 0;
    msg.angular.z =  (M_PI/4.0);
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
}

void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 50; i++) {
    ROS_INFO_STREAM("Robot Stop");
    msg.linear.x = 0;
    msg.angular.z =  0;
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
}
