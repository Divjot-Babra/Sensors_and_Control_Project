/*! @file
 *
 *  @brief 41014 Sensors & Controls - Fetch Following Project
 *
 *  This class contains the basic path following algoritm to allow fetch follow the guider object
 *
 *  @note this class is to be splited into other header and implementation files later
 *
 *  @author Zhifeng Huang
 *  @maintainer Zhifeng Huang
 *  @date 17-09-2022
 *  @note initial release 17-09-2022
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_datatypes.h" //To use getYaw function from the quaternion of orientation
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

//Global variables to receive info from rostopics
int Tracker_Status_ = 0;                  //Track the state of sensing
geometry_msgs::PoseStamped GuiderPose_;   //Return the guider 3D positions in the Fetch camera frame


//Callback to determine if the guider is detectable
void TrackerCallback (const std_msgs::Int8::ConstPtr& msg)
{
  Tracker_Status_ = msg.get()->data;
}

//Callback to retrieve guider pose in fetch camera's local frame
void GuiderCallback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  GuiderPose_.pose = msg.get()->pose;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_controller");
  ros::NodeHandle nh;

  /* Subscriber to rostopic, will need to create relative callback functions
   *
   * Priority is to fulfill sub 1&2 and pub for general sensing
   *
   * pub needs NO chattercallback
  */

  //sub1, visp object position, to know how far is the camera to the QR code and angle etc
  ros::Subscriber GuiderPosition_ = nh.subscribe("/visp_auto_tracker/object_position", 1000, GuiderCallback);

  //sub2, visp status, to determine whether the QR code/tracking object is within range
  ros::Subscriber GuiderState_ = nh.subscribe("/visp_auto_tracker/status", 1000, TrackerCallback);

  //sub3, fetch odmo, to know poses of fetch in world, will need it for pure pursuit(PID shouldn't need this)
  //sub4, fetch laser scan, for obstacle avoidance
  //...

  //pub, to publish linear and angular velocity to the fetch robot
  ros::Publisher FetchFollow = nh.advertise<geometry_msgs::Twist>("/cmd_vel_fetch", 1000);

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
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    //Below variables can use to enter different mode
    bool PathFollow = false;
    bool Collided = false;
    bool NearestWall = false;

    //To control lin and ang velocity of the fetch robot
    geometry_msgs::Twist Fetch;
    double FetchLin = 0.0;
    double FetchAng = 0.0;

    //Observe the camera signal
    ROS_INFO_STREAM("Status of VISP: " << Tracker_Status_);

    //0 means the camera is detecting, 1 means the QR is recognised
    if(Tracker_Status_ == 1 && Collided == false)
    {
      //If image can be deteched, start path following mode
      PathFollow = true;
      ROS_WARN("Enabling Path Follow Mode!");
    }

    else if (Tracker_Status_ == 3)
    {
      ROS_WARN("Fetch can see Guider!");

      //To get the relative angle of guider in the view of fetch camera
      double theta = atan2(GuiderPose_.pose.position.x, GuiderPose_.pose.position.z);
      theta = theta*180/M_PI;
      ROS_WARN("angle (in deg): [%f]", theta);

      //To obtain perpendicular distance between the fetch camera and guider
      double DistG2F = std::sqrt(std::pow(GuiderPose_.pose.position.x,2)+std::pow(GuiderPose_.pose.position.z,2));

      //If theta is within -4 to 4 (min 3 to look nice), then move straight
      //If 0 < theta < 4, guider turning left, fetch need to rotate CCW
      //If -4< theta < 0, guider turning right, fetch need to rotate CW
      //Fetch should maintain 0.3 from the guider -> 0.3 is roughly the width of a cube in gazebo
      //lin and ang should be around 0.1-0.4m/s -> do 0.2

//      if (theta >= -3 && theta <= 3)
//      {
//        if(GuiderPose_.pose.position.z <= 0.6)
//        {
//          ROS_WARN("Fetch Move forward");
//          FetchLin = 0.3;
//          FetchAng = 0;
//        }
//        else
//        {
//          ROS_WARN("Too close! Stop!");
//          FetchLin = 0;
//          FetchAng = 0;
//        }
//      }
//      else if (theta > 4)
//      {
//        ROS_WARN("Turning Left");
//        FetchLin = 0;
//        FetchAng = 0.15;
//      }
//      else if (theta < -4)
//      {
//        ROS_WARN("Turning Right");
//        FetchLin = 0;
//        FetchAng = -0.15;
//      }
      //Always maintain 0.8m (in sim) between fetch and guider

      ROS_INFO_STREAM("Perpenducular distance: " << DistG2F);
      if(DistG2F >= 0.25)
      {
//        ROS_WARN("Fetch Move forward");
//        FetchLin = 0.3;
////        FetchAng = 0;
        if(theta >= 2)
        {
          ROS_WARN("Fetch Turning Right");
          FetchLin = 0.2;
          FetchAng = -1.2;
        }
        else if (theta <= -2)
        {
          ROS_WARN("Fetch Turning Left");
          FetchLin = 0.2;
          FetchAng = 1.2;
        }
        else {
          ROS_WARN("Fetch Move forward");
          FetchLin = 0.3;
          FetchAng = 0;
        }
      }
      else if(DistG2F <= 0.25)
      {
        ROS_WARN("Fetch Move backward");
        FetchLin = -0.2;
        FetchAng = 0;
//        if(theta >= 2)
//        {
//          ROS_WARN("Fetch Turning Right");
//          FetchLin = -0.2;
//          FetchAng = 1.0;
//        }
//        else if (theta <= -2)
//        {
//          ROS_WARN("Fetch Turning Left");
//          FetchLin = -0.2;
//          FetchAng = -1.0;
//        }
//        else {
//          ROS_WARN("Fetch Move backward");
//          FetchLin = -0.4;
//          FetchAng = 0;
//        }
      }
      else if (DistG2F <= 0)
      {
        ROS_WARN("Fetch Stops");
        FetchLin = 0;
        FetchAng = 0;
//        if(theta >= 2)
//        {
//          ROS_WARN("Fetch Turning Right");
//          FetchLin = -0.2;
//          FetchAng = 1.0;
//        }
//        else if (theta <= -2)
//        {
//          ROS_WARN("Fetch Turning Left");
//          FetchLin = -0.2;
//          FetchAng = -1.0;
//        }
//        else {
//          ROS_WARN("Fetch Move backward");
//          FetchLin = -0.4;
//          FetchAng = 0;
//        }
      }
//        FetchAng = 3;
}
    //Below can use to eanble collision avoidance/wall follow mode
//    else
//    {
//      Collided = true;
//      ROS_WARN("Obstacle Detected!");
////      ROS_INFO_STREAM("Obstacle Detected!");
//    }

    Fetch.linear.x = FetchLin;
    Fetch.angular.z = FetchAng;

    FetchFollow.publish(Fetch);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
