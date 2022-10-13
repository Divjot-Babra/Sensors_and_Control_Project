/*! @file
 *
 *  @brief 41014 Sensors & Controls for Mechatronic Systems - Fetch Following Project
 *
 *  This class contains the path following algorithm for controlling the Fetch robot to follow a marker pattern.
 *
 *  @note This class will be split into a header and implementation file later.
 *
 *  @author Zhifeng Huang, Lee Madden & Divjot Babra
 *  @maintainer Zhifeng Huang, Lee Madden & Divjot Babra
 *  @date 25-09-2022
 *  @note Initial release 19-10-2022
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include <sstream>

#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/transform_datatypes.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include <cmath>

#define FOLLOWING_DISTANCE 0.3
#define OBSTACLE_DISTANCE 0.65

#define MAX_SENSOR_RANGE 25
#define ANGULAR_RESOLUTION 1/3

#define OBSTACLE_ANGLE_MIN 30
#define OBSTACLE_ANGLE_MAX 190

//struct {
//    double Linear;
//    double Angle;
//    double Perpendicular;
//    double Rotate;
//    double Forward;
//}   OBS;

double last_error_Lin = 0;
double last_error_Ang = 0;

// Global variables to receive info from rostopics
int Tracker_Status_ = 0;                  //Track the state of sensing
geometry_msgs::PoseStamped GuiderPose_;   //Return the guider 3D positions in the Fetch camera frame
sensor_msgs::LaserScan LaserScan_;        //Return LaserScans of laser mounted on Fetch

//Obstacle detection
bool Obstacle_Detected = false;       //Track the state of sensing obstacles
bool Obstacle_Dodged = false;
bool TurnLeft = false;
bool TurnRight = false;

//Wall follow
bool Wall_Follow = false;

double Obstacle_Distance = 0;       //Track the distance of closest obstacle
double Obstacle_Angle = 0;          //Track the angle of the closest obstacle



// States of the Fetch
int State = 0;

//enum State
//{
//    Finding_Guider,         //Finding Guider
//    Following_Guider,       //Following Guider
//    Avoiding_Obstacle      //Avoiding Obstace
//};



// Callbacks:

// Callback to determine if the guider is detectable
void TrackerCallback (const std_msgs::Int8::ConstPtr& msg)
{
  Tracker_Status_ = msg.get()->data;
}

// Callback to retrieve guider pose from Fetch camera
void GuiderCallback (const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  GuiderPose_.pose = msg.get()->pose;
}

// Callback to retrieve scan ranges from Fetch Scanner
void ScannerCallback (const sensor_msgs::LaserScan& msg)
{
  LaserScan_ = msg;
}

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



//"OTHER" Code
//Second attempt at obstacle detection

void DetectObstacle()
{
    double obstacle_distance_ = 25;
    double obstacle_angle_ = 0;

    // iterate through base scan ranges to find the closest range
    for (unsigned int i=0; i < LaserScan_.ranges.size(); i++)
    {

        if ( LaserScan_.ranges.at(i) < obstacle_distance_)
        {
            obstacle_distance_ =  LaserScan_.ranges.at(i);
            obstacle_angle_ = i* LaserScan_.angle_increment*180/M_PI;
        }
    }

    // if obstacle is too close then record distance and angle as obstacle has been detected
    if(obstacle_distance_ < OBSTACLE_DISTANCE && Obstacle_Dodged == false && obstacle_angle_ > OBSTACLE_ANGLE_MIN && obstacle_angle_ <= OBSTACLE_ANGLE_MAX)
    {
      ROS_WARN("OBSTACLE DETECTED! ");
        Obstacle_Detected = true;
        Obstacle_Distance = obstacle_distance_;
        Obstacle_Angle = obstacle_angle_;
    }

    // if obstacle isnt close then it isnt an obstacle
    else
    {
        Obstacle_Detected = false;

        Obstacle_Distance = 0;
        Obstacle_Angle = 0;
    }
}



//Main:

int main(int argc, char **argv)
{
  ros::init(argc, argv, "fetch_controller");
  ros::NodeHandle nh;

  // Subscriber1 - visp object pose, returns pose of marker pattern
  ros::Subscriber GuiderPosition_ = nh.subscribe("/visp_auto_tracker/object_position", 100, GuiderCallback);

  // Subscriber2 - visp status, returns range status of marker pattern
  ros::Subscriber GuiderState_ = nh.subscribe("/visp_auto_tracker/status", 100, TrackerCallback);

  // Subscriber3 - fetch odmo, to know poses of fetch in world, will need it for pure pursuit(PID shouldn't need this)
  // Subscriber4 - fetch laser scan, for obstacle avoidance
  ros::Subscriber Scanner_ = nh.subscribe("/base_scan", 1000, ScannerCallback);

  //any way to subscribe to odom?
  //ros::Subscriber Odom_ = nh.subscribe("/odom", )

  // Publisher1 - fetch velocity, to publish linear and angular velocity to the Fetch robot
  ros::Publisher FetchFollow = nh.advertise<geometry_msgs::Twist>("/cmd_vel_fetch", 1000);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
          // To control linear and angular velocity of the fetch robot
          geometry_msgs::Twist Fetch;
          double FetchLin = 0.0;
          double FetchAng = 0.0;

          // Observe the camera signal
          ROS_INFO_STREAM("Status of VISP: " << Tracker_Status_);

          ROS_WARN("CHECKING FOR OBSTABLES!");
          DetectObstacle();

          // 0 means the camera is in detecting mode, 1 means it is in tracking mode
          if((Tracker_Status_ == 0 || Tracker_Status_ == 1) && Obstacle_Detected == false)
          {
            // No obstacle and can't see the QR Code
            ROS_WARN("Where is the QR Code?");

            // Need to search for the QR Code

            //COMMENTED OUT IRL

//            if (TurnLeft == true)
//            {
//                ROS_WARN("SEARCHING RIGHT");
//                FetchAng = -0.25;
//                FetchLin = 0;

//            }

//            else if (TurnRight == true)
//            {

//              ROS_WARN("SEARCHING LEFT");
//              FetchAng = 0.25;
//              FetchLin = 0;
//            }

//            else
//            {
//              ROS_WARN("SEARCHING LEFT");
//              FetchAng = 0.25;
//              FetchLin = 0;
//            }

          }
          else if ((Tracker_Status_ == 0 || Tracker_Status_ == 1) && Obstacle_Detected == true)
          {
            ROS_WARN("QR code cannot be read");
            //Implement wallfollow after collision detection?
            //WallFollow();

            //COMMENTED OUT IRL


//            if (TurnLeft == true)
//            {
//                ROS_WARN("SEARCHING RIGHT");
//                FetchAng = -0.25;
//                FetchLin = 0;

//            }

//            else if (TurnRight == true)
//            {

//              ROS_WARN("SEARCHING LEFT");
//              FetchAng = 0.25;
//              FetchLin = 0;
//            }

//            else
//            {
//              ROS_WARN("SEARCHING LEFT");
//              FetchAng = 0.25;
//              FetchLin = 0;
//            }


          }

          // Path Following here
          else if (Tracker_Status_ == 3)
          {
            ROS_WARN("Fetch can see Guider!");

            // To get the relative angle of guider from Fetch's camera
            double theta = atan2(GuiderPose_.pose.position.x, GuiderPose_.pose.position.z);
            theta = theta*180/M_PI;
            ROS_WARN("angle (in deg): [%f]", theta);

            // To obtain perpendicular distance between the Fetch's camera and guider
            double DistG2F = std::sqrt(std::pow(GuiderPose_.pose.position.x,2)+std::pow(GuiderPose_.pose.position.z,2));

            //COMMENTED OUT IRL


//            // if obstacle is detected and it hasnt been dodged, start dodging
//            if (Obstacle_Detected == true && Obstacle_Dodged == false)
//            {

//              ROS_WARN("DODGING OBSTACLE");

//              // dodge right
//                if(Obstacle_Angle > OBSTACLE_ANGLE_MIN && Obstacle_Angle <= 110)
//                {
//                    ROS_WARN("DODGING RIGHT");
//                    TurnRight = true;
//                    TurnLeft = false;

//                  FetchLin = 0.6;
//                  FetchAng = -0.8;
////                  FetchAng = AngularPID(Obstacle_Angle, 0.2);
////                  FetchLin = LinearPID(Obstacle_Distance, 0.05);
//                }

//               // dodge left
//                else if (Obstacle_Angle > 110 && Obstacle_Angle <= OBSTACLE_ANGLE_MAX)
//                {
//                    ROS_WARN("DODGING LEFT");
//                    TurnLeft = true;
//                    TurnRight = false;

//                  FetchLin = 0.6;
//                  FetchAng = 0.8;
////                  FetchAng = -AngularPID(Obstacle_Angle, 0.2);
////                  FetchLin = LinearPID(Obstacle_Distance, 0.05);
//                }

//                // obstacle dodged set to true
//                else
//                {
//                Obstacle_Dodged = true;
//                break;
//                }
//              }

//            // when no obstacle is detected, just follow the guider
//            else if (Obstacle_Detected == false)
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
                  if (DistG2F >= 0.3)                         // Too far from marker pattern - move STRAIGHT
                  {
                    Switch = 1;
                  }
                  else if (DistG2F > 0.25 && DistG2F < 0.3)   // Perfect distance - STOP Fetch
                  {
                    Switch = 2;
                  }
                  else if (DistG2F > 0.1 && DistG2F <= 0.25)  // Too close to marker pattern - REVERSE
                  {
                    Switch = 3;
                  }

                  if ((theta < 2 && theta > -2) && Switch == 1)
                  {
                    ROS_WARN("Fetch Moving forward");
            //        FetchLin = 0.3;
                    FetchLin = LinearPID(DistG2F, 0.3);
                  }
                  else if (theta < -1.3 && Switch == 1)
                  {
                    ROS_WARN("Fetch Turning LEFT");
            //        FetchAng = 0.5;
                    FetchLin = LinearPID(DistG2F, 0.3);
                    FetchAng = AngularPID(theta, 2);

                    //TurnLeft = true;
                  }
                  else if (theta > 1.3 && Switch == 1)
                  {
                    ROS_WARN("Fetch Turning RIGHT");
            //        FetchAng = -0.5;
                    FetchLin = LinearPID(DistG2F, 0.3);
                    FetchAng = AngularPID(theta, 2);

                    //TurnRight = true;
                  }
                  else if (Switch == 3)
                  {
                    ROS_WARN("Fetch Moving BACKWARD");
            //        FetchLin = -0.3;
                    FetchLin = LinearPID(DistG2F, 0.3);
            //        FetchLin = -1 * FetchLin;
                  }

                  // Fetch is in range
                  else if (Switch == 2)
                  {
                    ROS_WARN("Fetch STOP");
                    FetchLin = 0;
                    FetchAng = 0;
                  }
//              }

//            }

          Fetch.linear.x = FetchLin;
          Fetch.angular.z = FetchAng;

          //Publish calculated velocities to fetch
          FetchFollow.publish(Fetch);

          Obstacle_Dodged = false;

          ros::spinOnce();
          loop_rate.sleep();


        }

        ros::spin();

        return 0;
}




