#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"

//geometry_msgs::PoseStamped fetchPose;

void chatterCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_controller");
  ros::NodeHandle nh;


  ros::Subscriber sub = nh.subscribe("chatter", 1000, chatterCallback);

  /* Subscriber to rostopic, will need to create relative callback functions
   *
   * Priority is to fulfill sub 1&2 and pub for general sensing
   *
   * pub needs NO chattercallback
  */

  //sub1, visp object position, to know how far is the camera to the QR code and angle etc
  //sub2, visp status, to determine whether the QR code/tracking object is within range
  //sub3, fetch odmo, to know poses of fetch in world, will need it for pure pursuit(PID shouldn't need this)
  //sub4, fetch laser scan, for obstacle avoidance

  //pub, to publish linear and angular velocity to the fetch robot

  /* General logics
   *
   * while ros ok
   *
   * check the status of visp
   * if within in range, start following (a boolean to initialiase movement)
   * else fetch spin 360 to find target
   *
   * if follow_bol is true, check the object position
   *
   * (Gazebo distance is not 1m, will need to test)
   * if distance is greater than 1m, move forward -> fetch lin = 0.5
   * if distance is less or equal to 1m, stop fetch -> fetch lin = 0
   *
   * (will need to test if positive is left)
   * if on the left, turn fetch left -> fetch ang = 0.5
   * if on the right, turn fetch right -> fetch ang = -0.5
   */



  ROS_WARN("WOROKS");
  ros::spin();

  return 0;
}
