#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "stdlib.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include <tf/transform_datatypes.h>
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <kobuki_msgs/BumperEvent.h>

// global constants
const int FRONT_EDGE_INDEX = 319;
const int LEFT_MIDDLE_EDGE_INDEX = 479;
const int LEFT_EDGE_INDEX = 639;

const double WALL_FRONT_SAFETY_DIST= 1;
const double WALL_FOLLOWING_DIST_MIN = 0.2;
const double WALL_FOLLOWING_DIST_MAX = 0.4;
const double WALL_FOLLOWING_LASER_DIST_MIN = 0.4;
const double WALL_FOLLOWING_LASER_DIST_MAX = 2.0;


// global variables
double front_edge = -1.0;
double left_edge = -1.0;
double closest_edge = -1.0;

double current_x = 0;
double current_y = 0;
double yaw_angle = 0;

bool hitWall = false;


// function prototypes
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double goal_x = 0.5);
void turnRight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void turnLeft(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
std::map<char, bool> detectWalls();
void goBack(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
// void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub);
// void correctYaw(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw_angle);


// callback functions
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
  std::vector<float> filted_ranges = scan->ranges;
  // replace all NaN with 99
  for (int i = FRONT_EDGE_INDEX; i <= LEFT_MIDDLE_EDGE_INDEX; i++) {
    if (std::isnan(filted_ranges[i])) {
      filted_ranges[i] = 99;
    }
  }

  int closest_edge_index = FRONT_EDGE_INDEX;
  for (int i = FRONT_EDGE_INDEX; i <= LEFT_MIDDLE_EDGE_INDEX; i++) {
    if ((filted_ranges[i] < filted_ranges[closest_edge_index])) {
      closest_edge_index = i;
    }
  }

  front_edge = filted_ranges[FRONT_EDGE_INDEX];
  left_edge = filted_ranges[LEFT_EDGE_INDEX];
  closest_edge = filted_ranges[closest_edge_index];
}

void poseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  current_x = msg->pose.pose.position.x;
  current_y = msg->pose.pose.position.y;
  yaw_angle = tf::getYaw(msg->pose.pose.orientation);
}

void bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg) {
  if (msg->state == kobuki_msgs::BumperEvent::PRESSED) {
    switch (msg->bumper) {
      case kobuki_msgs::BumperEvent::LEFT :
        hitWall = true;
        ROS_INFO_STREAM("HIT WALL");
      break;
      case kobuki_msgs::BumperEvent::CENTER :
        hitWall = true;
        ROS_INFO_STREAM("HIT WALL");
      break;
      case kobuki_msgs::BumperEvent::RIGHT :
        hitWall = true;
        ROS_INFO_STREAM("HIT WALL");
      break;
    }
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher twist_pub = n.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 100);

  ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);
  ros::Subscriber poseSub = n.subscribe<nav_msgs::Odometry>("/odom", 10, poseCallback);
  ros::Subscriber bumperSub = n.subscribe<kobuki_msgs::BumperEvent>("/mobile_base/events/bumper", 10, bumperCallback);

  geometry_msgs::Twist msg;

  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ros::spinOnce();

    if (hitWall) {
      stopRobot(msg ,loop_rate, twist_pub);
      goBack(msg ,loop_rate, twist_pub);
      hitWall = false;
    }

    if (left_edge != -1) {
      std::map<char, bool> wallMap = detectWalls();

      if (!wallMap.at('l')) {
        turnLeft(msg ,loop_rate, twist_pub);
      }
      else if (!wallMap.at('f')) {
        goStraight(msg ,loop_rate, twist_pub);
        ros::spinOnce();
      }
      else {
        turnRight(msg ,loop_rate, twist_pub);
        ros::spinOnce();
        goStraight(msg ,loop_rate, twist_pub, 0.3);
      }

      // TODO : 2. Include bumper controllers <-- collision detection
    }

    loop_rate.sleep();
  }

  return 0;
}

// COMPLETE!
void goStraight(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double goal_x) {
  // double goal_x = 1.0;
  double current_pos_x = 0.0;
  double linear_speed = 0.2;
  double angular_correction = 0.00604; // changed from 0.007551
  ros::WallTime t0 = ros::WallTime::now();

  do {
    // ROS_INFO("current_pos_x : %f", current_pos_x);
    msg.linear.x = linear_speed;
    msg.angular.z = angular_correction;
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

    // ROS_INFO("dt %f", t1.toSec() - t0.toSec());
    // ROS_INFO("turn_angle %Lf", turn_angle * 180.0 / M_PI);
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

    // ROS_INFO("dt %f", t1.toSec() - t0.toSec());
    // ROS_INFO("turn_angle %Lf", turn_angle * 180.0 / M_PI);
  } while(turn_angle > 0);

  msg.angular.z =  0;
  twist_pub.publish(msg);
}

// COMPLETE!
void stopRobot(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  ROS_INFO_STREAM("Robot Stop");
  for (int i = 0; i < 50; i++) {
    msg.linear.x = 0;
    msg.angular.z =  0;
    twist_pub.publish(msg);
    loop_rate.sleep();
  }
}

// COMPLETE!
std::map<char, bool> detectWalls() {
  std::map<char, bool> wallMap;

  // bool wallFront  = (front_edge < WALL_FRONT_SAFETY_DIST) ? true : false;
  bool wallMiddleLeft  = (closest_edge < WALL_FRONT_SAFETY_DIST) ? true : false;

  // reason for considering NaN as true :
  // NaN occurs when wall is either too close or far away to detect.
  // This robot follows the left wall, hence it cannot logically be too far away.
  // Hence logically, NaN occurance can be considered as the robot is very close to the left wall.
  bool wallLeft   = ((left_edge < WALL_FOLLOWING_LASER_DIST_MAX) || std::isnan(left_edge)) ? true : false;

  ROS_INFO("wallMiddleLeft : %d", wallMiddleLeft);
  ROS_INFO("wallLeft : %d", wallLeft);

  wallMap.insert(std::pair<char, bool>('f', wallMiddleLeft));
  wallMap.insert(std::pair<char, bool>('l', wallLeft));

  return wallMap;
}

void goBack(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
  double goal_x = 1.0;
  double current_pos_x = 0.0;
  double linear_speed = 0.2;
  double angular_correction = 0.00604; // changed from 0.007551
  ros::WallTime t0 = ros::WallTime::now();

  do {
    msg.linear.x = -linear_speed;
    msg.angular.z = -angular_correction;
    twist_pub.publish(msg);
    current_pos_x = linear_speed * (ros::WallTime::now().toSec() - t0.toSec());
  } while (current_pos_x < goal_x);

  stopRobot(msg ,loop_rate, twist_pub);
}

// void correctYaw(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub, double yaw_angle) {
//   if (abs(yaw_angle) > 0.2) {
//     ROS_INFO_STREAM("Correcting yaw");
//     double angular_speed = (M_PI)/288.0;
//     msg.linear.x = 0;
//     msg.angular.z = (yaw_angle > 0) ? -angular_speed : angular_speed;
//     long double wanted_angle = -yaw_angle;
//     long double turn_angle = 0.0;
//     ros::WallTime t0 = ros::WallTime::now();
//
//     do {
//       twist_pub.publish(msg);
//       ros::WallTime t1 = ros::WallTime::now();
//       turn_angle = wanted_angle - (t1.toSec() - t0.toSec())*angular_speed;
//       loop_rate.sleep();
//     } while(turn_angle > 0);
//
//     msg.angular.z =  0;
//     twist_pub.publish(msg);
//   }
// }

// COMPLETE!
// void correctCourse(geometry_msgs::Twist msg, ros::Rate loop_rate, ros::Publisher twist_pub) {
//   // get distance from the left wall (orientation consider later)
//   // if (distance < safe_distance_min)
//   //  turnRight()
//   //  goStraight(safe_distance_min - distance + 0.2)
//   //  turnLeft()
//   //  stopRobot()
//   // else if (distance > safe_distance_max || NaN) {
//   //  turnLeft()
//   //  goStraight(distance - safe_distance_max + 0.2)
//   //  turnRight()
//   //  stopRobot()
//
//   // BUG : I think when the wall is missed on the left, a nan is given. Nan is considered < WALL_FOLLOWING_LASER_DIST_MIN; hence right turn
//   // TODO: Check and rectify this; print left_edge and observe
//   if (left_edge < WALL_FOLLOWING_LASER_DIST_MIN) {
//     ROS_INFO_STREAM("turning right by correction");
//     turnRight(msg ,loop_rate, twist_pub);
//     double correction = (WALL_FOLLOWING_DIST_MIN - (left_edge * 0.5)); // 0.5 = Sin 30 deg
//     // ROS_INFO("correction : %f", correction);
//     goStraight(msg ,loop_rate, twist_pub, correction);
//     turnLeft(msg ,loop_rate, twist_pub);
//     stopRobot(msg ,loop_rate, twist_pub);
//   }
//   else if (left_edge > WALL_FOLLOWING_LASER_DIST_MAX || std::isnan(left_edge)) {
//     turnLeft(msg ,loop_rate, twist_pub);
//     double correction = ((left_edge * 0.5) - WALL_FOLLOWING_DIST_MAX); // 0.5 = Sin 30 deg
//     // ROS_INFO("correction : %f", correction);
//     goStraight(msg ,loop_rate, twist_pub, correction);
//     turnRight(msg ,loop_rate, twist_pub);
//     stopRobot(msg ,loop_rate, twist_pub);
//   }
// }
