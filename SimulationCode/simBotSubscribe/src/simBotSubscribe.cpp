#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
   ROS_INFO("Robot x,y,z: %f,%f,%f", msg->linear.x, msg->linear.y, msg->linear.z);
}


int main(int argc, char *argv[])
{
   ros::init(argc, argv, "simBotSubscriber", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
   ROS_INFO("A simple subscriber node");

   geometry_msgs::Twist robot_twist;

   ros::NodeHandle handle;

   ros::Subscriber twistSub = handle.subscribe("cmd_vel", 1, twistCallback);
   ROS_INFO("Num publishers: %d", twistSub.getNumPublishers());

   ros::Rate loop_rate(100);
   while(ros::ok())
   {
      ros::spinOnce();
      loop_rate.sleep();
   }

   return 0;
}

   
