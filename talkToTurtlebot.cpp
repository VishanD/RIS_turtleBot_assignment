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
const double WALL_DETECTION_DIST = 3.8;


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
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
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
}

void ComPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  yaw = tf::getYaw(msg->pose.pose.orientation);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
  ros::Subscriber ComPose_sub = n.subscribe("/odom", 10, ComPoseCallback);

  geometry_msgs::Twist msg;

  ros::Rate loop_rate(5);

  while (ros::ok())
  {
    ROS_INFO("left  : %f", left_edge);
    // ROS_INFO("front : %f", front_edge);
    // ROS_INFO("right : %f", right_edge);
    // ROS_INFO("yaw : %f", yaw);

    // checking for NaN and replacing it with an arbitary value
    if (isnan(front_edge)) {
      front_edge = 2;
    }

    // go straight if left side and front is clear
    if (/*left_edge > SIDE_SAFETY_MARGIN && */front_edge > FRONT_SAFETY_MARGIN) {
      goStraight(msg, loop_rate, twist_pub);
    }

    // turn right if front is blocked and right side is clear
    else if (front_edge < FRONT_SAFETY_MARGIN) {
      turnRight(msg, loop_rate, twist_pub);
    }

    if (left_edge >= WALL_DETECTION_DIST) {
      stopRobot(msg, loop_rate, twist_pub);
      ROS_INFO_STREAM("Entered door finding function");
      turnLeft(msg, loop_rate, twist_pub);
      goStraight(msg, loop_rate, twist_pub);
    }

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

void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 10; i++) {
    ROS_INFO_STREAM("Turn right");
    msg.linear.x = 0;
    msg.angular.z =  -(M_PI/4.0);
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
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

// void reverseRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
//   for (int i = 0; i < 10; i++) {
//     ROS_INFO_STREAM("reverse");
//     msg.linear.x = -0.1;
//     msg.angular.z =  0;
//     twist_pub.publish(msg);
//     loop_rate.sleep();
//   }
// }

void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  ROS_INFO_STREAM("Robot Stop");
  msg.linear.x = 0;
  msg.angular.z =  0;
  twist_pub.publish(msg);
}

// THE MOST IMPORTANT FUNCTION!
// void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw) {
//   for (int i = 0; i < 5; i++) {
//     ROS_INFO_STREAM("Stop Robot");
//     stopRobot(msg, loop_rate, twist_pub);
//     // correct yaw
//     msg.linear.x = 0.0;
//     msg.angular.z = -yaw;
//     ROS_INFO("Yaw: %f; Corrected Yaw: %f", yaw, msg.angular.z);
//
//     // ALGORITHM
//     /*
//     If left_edge > WALL_FOLLOWING_DIST <---check condition in main loop
//         stop
//         see the yaw value
//         correct yaw so that robot is having 0 rad yaw
//     */
//
//     // if (left_edge > WALL_FOLLOWING_DIST) {
//     //   turnLeft(msg, loop_rate, twist_pub);
//     //   // goStraight(msg, loop_rate, twist_pub);
//     // }
//     // msg.linear.x = 0.0;
//     // msg.angular.z = (((M_PI/2.0) - yaw)/5.0);
//     // ROS_INFO("Yaw: %f", yaw);
//     // ROS_INFO("angular_correction: %f", msg.angular.z);
//     twist_pub.publish(msg);
//     loop_rate.sleep();
//   }
//
//   // for (int i = 0; i < 5; i++) {
//   //   if (left_edge <= 0.2) {
//   //     break;
//   //   }
//   //   else {
//   //     ROS_INFO_STREAM("correcting drift");
//   //     msg.linear.x = 0.2;
//   //     msg.angular.z = 0;
//   //     twist_pub.publish(msg);
//   //     loop_rate.sleep();
//   //   }
//   // }
// }

// void initializeRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
//   ROS_INFO("WALL_FOLLOWING_DIST %f", WALL_FOLLOWING_DIST);
//   ROS_INFO("left_edge %f", left_edge);
//   int iterations = ceil((WALL_FOLLOWING_DIST - left_edge)/0.2)*5;
//   ROS_INFO("no of iterations = %d", iterations);
//   turnRight(msg, loop_rate, twist_pub);
//
//   for (int i = 0; i < iterations; i++) {
//     goStraight(msg, loop_rate, twist_pub);
//     loop_rate.sleep();
//   }
//
//   turnLeft(msg, loop_rate, twist_pub);
//   stopRobot(msg, loop_rate, twist_pub);
// }
