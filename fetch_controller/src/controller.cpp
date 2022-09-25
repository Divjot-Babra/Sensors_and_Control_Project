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

#define FOLLOWING_DISTANCE 0.3
double last_error_Lin = 0;
double last_error_Ang = 0;

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


double LinearPID(double CurrentDist, double TargetDist)
{
  double KP = -2.5;
//  double KI = 0;
  double KD = 1.4;

  double error = TargetDist - CurrentDist;

//  double integrals += error;
  double derivative = error - last_error_Lin;

  double Linvel = KP*error + KD*derivative;

  last_error_Lin = error;

  return Linvel;
}

double AngularPID(double CurrentAng, double TargetAng)
{
  double KP = 0.1;
//  double KI = 0;
  double KD = 0.05;

  double error = TargetAng - CurrentAng;
  double derivative = error - last_error_Ang;

  double Angvel = KP*error + KD*derivative;

  last_error_Ang = error;

  return Angvel;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_controller");
  ros::NodeHandle nh;

  //sub1, visp object position, to know how far is the camera to the QR code and angle etc
  ros::Subscriber GuiderPosition_ = nh.subscribe("/visp_auto_tracker/object_position", 100, GuiderCallback);

  //sub2, visp status, to determine whether the QR code/tracking object is within range
  ros::Subscriber GuiderState_ = nh.subscribe("/visp_auto_tracker/status", 100, TrackerCallback);

  //sub3, fetch odmo, to know poses of fetch in world, will need it for pure pursuit(PID shouldn't need this)
  //sub4, fetch laser scan, for obstacle avoidance

  //pub, to publish linear and angular velocity to the fetch robot
  ros::Publisher FetchFollow = nh.advertise<geometry_msgs::Twist>("/cmd_vel_fetch", 1000);

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

    //Path Following here
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
      //Always maintain 0.8m (in sim) between fetch and guider

      int Switch = 0;
      if (DistG2F >= 0.3)                         //Follow in STRAIGHT path
      {
        Switch = 1;
      }
      else if (DistG2F > 0.25 && DistG2F < 0.3)   //Perfect distance, fetch STOP
      {
        Switch = 2;
      }
      else if (DistG2F > 0.1 && DistG2F <= 0.25)  //Too close to guider, REVERSE
      {
        Switch = 3;
      }

      if ((theta < 2 && theta > -2) && Switch == 1)
      {
        ROS_WARN("Fetch Moving forward");
//        FetchLin = 0.3;
        FetchLin = LinearPID(DistG2F, 0.3);
      }
      else if (theta < -2 && Switch == 1)
      {
        ROS_WARN("Fetch Turning LEFT");
//        FetchAng = 0.5;
        FetchLin = LinearPID(DistG2F, 0.3);
        FetchAng = AngularPID(theta, 2);
      }
      else if (theta > 2 && Switch == 1)
      {
        ROS_WARN("Fetch Turning RIGHT");
//        FetchAng = -0.5;
        FetchLin = LinearPID(DistG2F, 0.3);
        FetchAng = AngularPID(theta, 2);
      }
      else if (Switch == 3)
      {
        ROS_WARN("Fetch Moving BACKWARD");
//        FetchLin = -0.3;
        FetchLin = LinearPID(DistG2F, 0.3);
//        FetchLin = -1 * FetchLin;
      }
      else if (Switch == 2)
      {
        ROS_WARN("Fetch STOP");
        FetchLin = 0;
        FetchAng = 0;
      }
    }

    Fetch.linear.x = FetchLin;
    Fetch.angular.z = FetchAng;

    FetchFollow.publish(Fetch);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}


// BasicPursuit function
//{
//}

//// pure pursuit algo that determines length of arc.
//void PurePursuit (double distance, double angle)
//{
//    double y;
//    double x;

//    y = 2*x/sqrt(l);

//    //ROBOT rotates to face the qr code
//    //robot moves forward toward qr code

//    //arc = ( 2 * x ) / sqrt (distance)

//    //angular vel = curvature * linear vel ~ w = y *

//    // need to determine angular and linear velocity from this

//    // return arc
//}

//To be implemented after PP
//void PID (/*intakes distance readings*/)
//{

//}

//void collisionavoidance
//{
//
//}


