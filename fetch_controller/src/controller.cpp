/*! @file
 *  @brief 41014 Sensors & Control for Mechatronic Systems - Fetch Following Project
 *  This file contains the path following algorithm for controlling the Fetch robot to follow a marker pattern.
 *  @note Obstacle detection and avoidance functionality has been commented out for faster demonstration.
 *  To enable obstacle detection and avoidance functionality, uncomment the following lines:
 *  164 - 165
 *  235 - 269
 *  325
 *  @author Zhifeng Huang, Lee Madden & Divjot Babra
 *  @date 18-10-2022
 *  @version 5.0
 */

#include <sstream>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

#define FOLLOWING_DISTANCE 0.3
#define OBSTACLE_DISTANCE 0.65
#define MAX_SENSOR_RANGE 25
#define ANGULAR_RESOLUTION 1/3
#define OBSTACLE_ANGLE_MIN 30
#define OBSTACLE_ANGLE_MAX 190

double last_error_Lin = 0;
double last_error_Ang = 0;

// Global variables to receive info from rostopics
int Tracker_Status_ = 0; // Track the state of sensing
geometry_msgs::PoseStamped GuiderPose_; // Return the guider 3D position in the Fetch camera frame
sensor_msgs::LaserScan LaserScan_; // Return laser readings from base scanner on Fetch

// Variables for obstacle detection and avoidance
bool Obstacle_Detected = false;
bool Obstacle_Dodged = false;
bool TurnLeft = false;
bool TurnRight = false;
double Obstacle_Distance = 0; // For tracking the distance of closest obstacle
double Obstacle_Angle = 0; // For tracking the angle of the closest obstacle

// For storing state of Fetch
int State = 0;

// --- Callbacks ---
void TrackerCallback(const std_msgs::Int8::ConstPtr& msg) // Callback to determine if the guider is detectable
{
  Tracker_Status_ = msg.get()->data;
}

void GuiderCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) // Callback to retrieve guider pose from Fetch camera
{
  GuiderPose_.pose = msg.get()->pose;
}

void ScannerCallback(const sensor_msgs::LaserScan& msg) // Callback to retrieve scan ranges from Fetch Scanner
{
  LaserScan_ = msg;
}

// --- Functions ---
double LinearPID(double CurrentDist, double TargetDist)
{
  double KP = -2.2;
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
  double KP = 0.077;
//  double KI = 0;
  double KD = 0.125;

  double error = TargetAng - CurrentAng;

  double derivative = error - last_error_Ang;

  double Angvel = KP*error + KD*derivative;

  last_error_Ang = error;

  return Angvel;
}

void DetectObstacle()
{
    double obstacle_distance_ = 25;
    double obstacle_angle_ = 0;

    // Iterate through base scan ranges to find the closest obstacle
    for(unsigned int i = 0; i < LaserScan_.ranges.size(); i++)
    {
        if(LaserScan_.ranges.at(i) < obstacle_distance_)
        {
            obstacle_distance_ =  LaserScan_.ranges.at(i);
            obstacle_angle_ = i*LaserScan_.angle_increment*180/M_PI;
        }
    }

    // If obstacle is too close then record distance and angle from the closest obstacle
    if(obstacle_distance_ < OBSTACLE_DISTANCE && Obstacle_Dodged == false && obstacle_angle_ > OBSTACLE_ANGLE_MIN && obstacle_angle_ <= OBSTACLE_ANGLE_MAX)
    {
        ROS_WARN("OBSTACLE DETECTED!");
        Obstacle_Detected = true;
        Obstacle_Distance = obstacle_distance_;
        Obstacle_Angle = obstacle_angle_;
    }
    else
    {
        Obstacle_Detected = false;
        Obstacle_Distance = 0;
        Obstacle_Angle = 0;
    }
}

// --- Main Program ---
int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_controller");
  ros::NodeHandle nh;

  // Subscriber1 - visp object pose, returns pose of marker pattern
  ros::Subscriber GuiderPosition_ = nh.subscribe("/visp_auto_tracker/object_position", 100, GuiderCallback);

  // Subscriber2 - visp status, returns tracking status of marker pattern
  ros::Subscriber GuiderState_ = nh.subscribe("/visp_auto_tracker/status", 100, TrackerCallback);

  // Subscriber3 - fetch laser scan, for obstacle avoidance
  ros::Subscriber Scanner_ = nh.subscribe("/base_scan", 1000, ScannerCallback);

  // Publisher1 - fetch velocity, to publish linear and angular velocity to the Fetch robot
  ros::Publisher FetchFollow = nh.advertise<geometry_msgs::Twist>("/cmd_vel_fetch", 1000);

  ros::Rate loop_rate(10); // Setting communication frequency

  while(ros::ok())
  {
      // To control linear and angular velocity of the fetch robot
      geometry_msgs::Twist Fetch;
      double FetchLin = 0.0;
      double FetchAng = 0.0;

      ROS_INFO_STREAM("Status of VISP: " << Tracker_Status_);

      // ROS_WARN("CHECKING FOR OBSTABLES!");
      // DetectObstacle();

      // 0 means the camera is in detecting mode, 1 means it is in tracking mode
      if((Tracker_Status_ == 0 || Tracker_Status_ == 1) && Obstacle_Detected == false)
      {
          // No obstacle and cannot see the QR Code
          ROS_WARN("Where is the QR Code?");

          // Need to search for the QR Code
          if(TurnLeft == true)
          {
              ROS_WARN("SEARCHING RIGHT");
              FetchAng = -0.25;
              FetchLin = 0;
          }

          else if(TurnRight == true)
          {
              ROS_WARN("SEARCHING LEFT");
              FetchAng = 0.25;
              FetchLin = 0;
          }

          else // Search left by default
          {
              ROS_WARN("SEARCHING LEFT");
              FetchAng = 0.25;
              FetchLin = 0;
          }
      }
      else if((Tracker_Status_ == 0 || Tracker_Status_ == 1) && Obstacle_Detected == true)
      {
          ROS_WARN("QR code cannot be read");

          if(TurnLeft == true)
          {
              ROS_WARN("SEARCHING RIGHT");
              FetchAng = -0.25;
              FetchLin = 0;
          }

          else if(TurnRight == true)
          {
              ROS_WARN("SEARCHING LEFT");
              FetchAng = 0.25;
              FetchLin = 0;
          }

          else // Searching left by default
          {
              ROS_WARN("SEARCHING LEFT");
              FetchAng = 0.25;
              FetchLin = 0;
          }
      }
      // Path following here
      else if(Tracker_Status_ == 3)
      {
          ROS_WARN("Fetch can see guider!");

          // To get the relative angle of guider from Fetch's camera
          double theta = atan2(GuiderPose_.pose.position.x, GuiderPose_.pose.position.z);
          theta = theta*180/M_PI;
          ROS_WARN("Angle (in degrees): [%f]", theta);

          // To obtain direct distance between the Fetch's camera and guider
          double DistG2F = std::sqrt(std::pow(GuiderPose_.pose.position.x,2)+std::pow(GuiderPose_.pose.position.z,2));

//            // If obstacle is detected and it has not been dodged, start dodging
//            if(Obstacle_Detected == true && Obstacle_Dodged == false)
//            {
//              ROS_WARN("DODGING OBSTACLE");
//
//              // Dodge right
//              if(Obstacle_Angle > OBSTACLE_ANGLE_MIN && Obstacle_Angle <= 110)
//              {
//                 ROS_WARN("DODGING RIGHT");
//                 TurnRight = true;
//                 TurnLeft = false;
//                 FetchLin = 0.6;
//                 FetchAng = -0.8;
////                  FetchAng = AngularPID(Obstacle_Angle, 0.2);
////                  FetchLin = LinearPID(Obstacle_Distance, 0.05);
//              }
//              // Dodge left
//              else if(Obstacle_Angle > 110 && Obstacle_Angle <= OBSTACLE_ANGLE_MAX)
//              {
//                 ROS_WARN("DODGING LEFT");
//                 TurnLeft = true;
//                 TurnRight = false;
//                 FetchLin = 0.6;
//                 FetchAng = 0.8;
////                  FetchAng = -AngularPID(Obstacle_Angle, 0.2);
////                  FetchLin = LinearPID(Obstacle_Distance, 0.05);
//                }
//              // Obstacle dodged
//              else
//              {
//                 Obstacle_Dodged = true;
//                 break;
//              }
//            }
//            // When no obstacle is detected, just follow the guider
//            else if(Obstacle_Detected == false)
//            {
              ROS_WARN("NO NEED TO DODGE");

              // If theta is within -4 to 4 (min 3 to look nice), then move straight
              // If 0 < theta < 4, guider turning left, Fetch need to rotate CCW
              // If -4 < theta < 0, guider turning right, Fetch need to rotate CW
              // Fetch should maintain 0.3 from the guider -> 0.3 is roughly the width of a cube in Gazebo
              // lin and ang should be around 0.1-0.4m/s -> do 0.2
              // Always maintain 0.8m (in Gazebo) between Fetch and marker pattern

              TurnLeft = false;
              TurnRight = false;

              int Switch = 0;
              if(DistG2F >= 0.3)                         // Too far from marker pattern - move STRAIGHT
              {
                  Switch = 1;
              }
              else if(DistG2F > 0.25 && DistG2F < 0.3)   // Perfect distance - STOP Fetch
              {
                  Switch = 2;
              }
              else if(DistG2F > 0.1 && DistG2F <= 0.25)  // Too close to marker pattern - REVERSE
              {
                  Switch = 3;
              }

              if((theta < 2 && theta > -2) && Switch == 1)
              {
                  ROS_WARN("Fetch moving forward");
                  FetchLin = LinearPID(DistG2F, 0.3);
              }
              else if(theta < -1.3 && Switch == 1)
              {
                  ROS_WARN("Fetch turning LEFT");
                  FetchLin = LinearPID(DistG2F, 0.3);
                  FetchAng = AngularPID(theta, 2);
              }
              else if(theta > 1.3 && Switch == 1)
              {
                  ROS_WARN("Fetch turning RIGHT");
                  FetchLin = LinearPID(DistG2F, 0.3);
                  FetchAng = AngularPID(theta, 2);
              }
              else if (Switch == 3)
              {
                  ROS_WARN("Fetch Moving BACKWARD");
                  FetchLin = LinearPID(DistG2F, 0.3);
              }
              // Fetch is in range
              else if (Switch == 2)
              {
                  ROS_WARN("Fetch STOP");
                  FetchLin = 0;
                  FetchAng = 0;
              }
//            }
  }
      Fetch.linear.x = FetchLin;
      Fetch.angular.z = FetchAng;

      FetchFollow.publish(Fetch); // Publish calculated velocities to fetch

      Obstacle_Dodged = false; // Reset obstacle detection status

      ros::spinOnce();
      loop_rate.sleep();
}
  ros::spin();
  return 0;
}
