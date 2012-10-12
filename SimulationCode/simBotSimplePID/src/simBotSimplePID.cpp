#include <ros/ros.h>
#include <geometrymsgs/Twist.h>

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "simBotSimplePID");
   ros::INFO("Simple PID position controller");

   ros::NodeHandle nodeHandle;

   float kp, kd, ki;
   float desired_position;

   geometry::msgs cmd_vel;

   

   return 0;   
}
