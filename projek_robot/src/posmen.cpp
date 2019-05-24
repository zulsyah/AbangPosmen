/** ROBOT POSMEN V1.0 BY GROUP 1

PROGRAMMER : ZULFHANIZAM AMIR SYAHPUTRA

**/

/** include all C/C++ library **/

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <opencv2/opencv.hpp>

/** define Item and Job ID **/
#define SMARTPHONE 10
#define HANDBAG 11
#define NO_CUSTOMER 13
#define HAS_CUSTOMER 12

using namespace std;

std::string path_to_sounds;

/** function declarations **/
bool moveToGoal(double goalXcoor, double goalYcoor);
void objectCallback();

/** declare the coordinates of houses**/
double house1_xcoor = 8.72123;
double house1_ycoor = -2.05946;

double house2_xcoor = 9.59979;
double house2_ycoor = -2.18837;

double initPose_x = 7.53366;
double initPose_y = -1.97713;

bool goalReached = false;

int id=0;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case SMARTPHONE:
         ROS_INFO("JOB Received! Navigate to customer house! (JOB ID : 10)");
         goalReached = moveToGoal(house1_xcoor, house1_ycoor);
         if (id == 12){
           ROS_INFO("Customer has pickup the item and thus Robot Posmen returned back to post station");
         }
         else{
           ROS_INFO("Customer has not pickup the item and thus Robot Posmen returned back to post station");
         }
         break;
      case HANDBAG:
         ROS_INFO("JOB Received! Navigate to customer house! (JOB ID : 11)");
         goalReached = moveToGoal(house2_xcoor, house2_ycoor);
         if (id == 12){
           ROS_INFO("Customer has pickup the item and thus Robot Posmen returned back to post station");
         }
         else{
           ROS_INFO("Customer has not pickup the item and thus Robot Posmen returned back to post station");
         }
         break;
      /**
      case HAS_CUSTOMER:
         ROS_INFO("Customer has pickup the iteam and thus Robot Posmen returned back to station");
         break;
      case NO_CUSTOMER:
         ROS_INFO("Customer has not pickup the iteam and thus Robot Posmen returned back to station");
         break;
      **/
      default: // other object
         ROS_INFO("JOB ID is invalid!");
      }
   }
   else
   {
      // No object detected
      ROS_INFO("No JOB ID Detected!");
   }
}

 int main(int argc, char** argv){
   ros::init(argc, argv, "map_navigation_node");
   ros::NodeHandle n;
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);

   while (ros::ok())
   {
      ROS_INFO("Scan your JOB ID in front of robot camera");
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}
bool moveToGoal(double goalXcoor, double goalYcoor){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "map";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  goalXcoor;
   goal.target_pose.pose.position.y =  goalYcoor;
   goal.target_pose.pose.position.z =  0.0;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Robot posmen is navigate to customer house ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("Robot posmen have arrived at customer house!");

      goalReached = moveToGoal(initPose_x, initPose_y);
      ROS_INFO("Robot posmen will returned to the post station.");

      return true;
   }
   else{
      ROS_INFO("Robot posmen have not arrived at customer house!");
      return false;
   }



}
