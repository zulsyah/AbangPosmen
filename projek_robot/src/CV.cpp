#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

#define SMILE 1
#define ARROW_LEFT 2
#define ARROW_UP 9
#define ARROW_DOWN 6

int id = 0;
ros::Publisher action_pub;
geometry_msgs::Twist set_vel;

void objectCallback(const std_msgs::Float32MultiArrayPtr &object)
{
   if (object->data.size() > 0)
   {
      id = object->data[0];

      switch (id)
      {
      case ARROW_LEFT:
         set_vel.linear.x = 0;
         set_vel.angular.z = 1;
         break;
      case ARROW_UP:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         ROS_INFO_STREAM("Job received! JOB ID is 9");
         break;
      case ARROW_DOWN:
         set_vel.linear.x = -1;
         set_vel.angular.z = 0;
         break;
      default: // other object
         set_vel.linear.x = 0;
         set_vel.angular.z = 0;
      }
      action_pub.publish(set_vel);
   }
   else
   {
      // No object detected
      set_vel.linear.x = 0;
      set_vel.angular.z = 0;
      action_pub.publish(set_vel);
   }
}

int main(int argc, char **argv)
{

   ros::init(argc, argv, "action_controller");
   ros::NodeHandle n("~");
   ros::Rate loop_rate(50);
   ros::Subscriber sub = n.subscribe("/objects", 1, objectCallback);
   action_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

   ROS_INFO_STREAM("WAITING FOR JOB");

   while (ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }
}
