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
const double FRONT_SAFETY_MARGIN = 1;


// global variables
double front_edge = 0;
double right_edge = 0;
double right_middle_edge = 0;
double left_edge = 0;
double left_middle_edge = 0;
double yaw = 0;


// function prototypes
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);


// callback functions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  right_edge = scan->ranges[RIGHT_EDGE_INDEX];
  right_middle_edge = scan->ranges[RIGHT_MIDDLE_EDGE_INDEX];
  front_edge = scan->ranges[FRONT_EDGE_INDEX];
  left_edge = scan->ranges[LEFT_EDGE_INDEX];
  left_middle_edge = scan->ranges[LEFT_MIDDLE_EDGE_INDEX];
  // ROS_INFO("Laser Scan Recieved.");
  // ROS_INFO_STREAM(scan->header.frame_id);
  // ROS_INFO("%s %f", "range_min       : ", scan->range_min);
  // ROS_INFO("%s %f", "range_max       : ", scan->range_max);
  // for (int i = 0; i < scan->ranges.size(); i++)
  // {
  //   if (i == 0) {
  //     // ROS_INFO("%s %f", "Right Range : ", scan->ranges[i]);
  //   }
  //   else if (i == (scan->ranges.size()/2)) {
  //     // ROS_INFO("%s %f", "Front Range : ", scan->ranges[i]);
  //   }
  //   else if (i == (scan->ranges.size() - 1)) {
  //     ROS_INFO("%s %f", "Left Range : ", scan->ranges[i]);
  //     // ROS_INFO("Yaw: [%f]", yaw);
  //   }
  // }

}

void ComPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  //     ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  //     ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  //
  // float linearposx=msg->pose.pose.position.x;
  // float linearposy=msg->pose.pose.position.y;
  double quatx= msg->pose.pose.orientation.x;
  double quaty= msg->pose.pose.orientation.y;
  double quatz= msg->pose.pose.orientation.z;
  double quatw= msg->pose.pose.orientation.w;

  tf::Quaternion q(quatx, quaty, quatz, quatw);
  tf::Matrix3x3 m(q);
  double roll, pitch;
  m.getRPY(roll, pitch, yaw);

  return;
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
    ROS_INFO("front : %f", front_edge);
    ROS_INFO("right : %f", right_edge);

    // checking for NaN and replacing it with an arbitary value
    if (front_edge != front_edge) {
      front_edge = 10;
    }

    // go straight if left side and front is clear
    if (left_edge > SIDE_SAFETY_MARGIN && front_edge > FRONT_SAFETY_MARGIN) {
      ROS_INFO_STREAM("Calling go straight");
      goStraight(msg, loop_rate, twist_pub);
    }

    // turn right if front is blocked and right side is clear

    // TODO : scanning just dead ahead misses obstacles at close range;
    // solution : to scan the left and right middle as well and turn
    else if (((front_edge < FRONT_SAFETY_MARGIN) || left_middle_edge < SIDE_SAFETY_MARGIN) && right_edge > SIDE_SAFETY_MARGIN) {
      turnRight(msg, loop_rate, twist_pub);
    }

    if (left_edge > 1) {
      correctCourse(msg, loop_rate, twist_pub);
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 10; i++) {
    if (front_edge > FRONT_SAFETY_MARGIN) {
      ROS_INFO_STREAM("Go straight");
      msg.linear.x = 0.2;
      msg.angular.z = 0;
      // twist_pub.publish(msg);
      // loop_rate.sleep();
    }
    else {
      ROS_INFO_STREAM("Obstacle ahead");
      msg.linear.x = 0.0;
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

void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  for (int i = 0; i < 5; i++) {
    ROS_INFO_STREAM("correcting drift");

    // HACK : Put 1 for 1m; declare as a constant above
    if (left_edge > 1) {
      turnLeft(msg, loop_rate, twist_pub);
      goStraight(msg, loop_rate, twist_pub);
    }
    msg.linear.x = 0.0;
    msg.angular.z = (((M_PI/2.0) - yaw)/5.0);
    twist_pub.publish(msg);
    loop_rate.sleep();
  }

  // for (int i = 0; i < 5; i++) {
  //   if (left_edge <= 0.2) {
  //     break;
  //   }
  //   else {
  //     ROS_INFO_STREAM("correcting drift");
  //     msg.linear.x = 0.2;
  //     msg.angular.z = 0;
  //     twist_pub.publish(msg);
  //     loop_rate.sleep();
  //   }
  // }
}
