#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

float current_x;
const float output_speed = 5.0f;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   current_x = msg->pose.pose.position.x;
}

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "simBotSimplePID");
   ROS_INFO("Simple PID position controller");

   ros::NodeHandle nodeHandle;

   float desired_position = 3.0f;

   // PID tuning consts
   float kp = 10.0f, kd = 0.0f, ki = 0.0f;

   float old_x = 0.0f;
   float err_sum = 0.0f;
   float dx = 0.0f;

   geometry_msgs::Twist cmd_vel;
   nav_msgs::Odometry odom;  
   
   ros::Subscriber odom_sub = nodeHandle.subscribe("base_pose_ground_truth", 1000, odomCallback);
   ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
   
   ros::Rate loop_rate(100);
   while(ros::ok())
   {
      ros::spinOnce();

      // PID terms
      float error = desired_position - current_x;
      dx = current_x - old_x;
      err_sum += error;

      float output = error * kp + dx * kd + err_sum * ki;

      if(output > output_speed) output = output_speed;
      else if(output < -output_speed) output = -output_speed;

      cmd_vel.linear.x = output;

      cmd_vel_pub.publish(cmd_vel);
      
      loop_rate.sleep();
   } 
   
   return 0;   
}
