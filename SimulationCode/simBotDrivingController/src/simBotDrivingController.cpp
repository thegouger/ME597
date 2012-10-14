#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

const float MAX_OUTPUT_SPEED = 5.0f;

struct vector2d
{
   float x;
   float y;
};

float vector2dMagnitude(vector2d v) { return sqrt(v.x*v.x + v.y*v.y); }

vector2d current_pos;

void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   current_pos.x = msg->pose.pose.position.x;
   current_pos.y = msg->pose.pose.position.y;
}

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "simBotDrivingController");
   ROS_INFO("Combined velocity and steering controller");

   ros::NodeHandle nodeHandle;


   // desired travel velocity
   float desired_velocity = 2.0f;

   // desired waypoints
   vector2d waypoints[4]; 
   waypoints[0] = {1.0f, -1.0f};
   waypoints[1] = {4.0f, -1.0f};
   waypoints[2] = {4.0f, 2.0f};
   waypoints[3] = {1.0f, -1.0f};
   int way_state = 1;

   // PID tuning consts
   float kp = 10.0f, kd = 0.0f, ki = 0.0f;

   // Stanley constant
   float ks = 1.0f;

   // PID, steering intermediaries
   vector2d old_pos = {0.0f, 0.0f}, old_vel = {0.0f, 0.0f};
   float err_sum = 0.0f;

   // msgs to send/receive
   geometry_msgs::Twist cmd_vel;
   nav_msgs::Odometry odom;

   // subscribe to odom, publish cmd_vel
   ros::Subscriber odom_sub   = nodeHandle.subscribe("base_pose_ground_truth", 1, odomCallback);
   ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   ros::Rate loop_rate(100); // be sure to use sim time if simulating
   while(ros::ok())
   {
      ros::spinOnce();

      // PID controller

      // current velocity in units/sec
      vector2d vel;
      vel.x = (current_pos.x - old_pos.x) * 100.0f;
      vel.y = (current_pos.x - old_pos.y) * 100.0f;

      // PID terms
      float error = desired_velocity - vector2dMagnitude(vel);
      err_sum += error;
      float dv = (vector2dMagnitude(vel) - vector2dMagnitude(old_vel))*100.0f;

      float vel_output = error * kp + dv * kd + err_sum * ki;

      // limit velocity output
      if(vel_output > MAX_OUTPUT_SPEED)       vel_output = MAX_OUTPUT_SPEED;
      else if(vel_output < -MAX_OUTPUT_SPEED) vel_output = -MAX_OUTPUT_SPEED;

      cmd_vel.linear.x = vel_output;


      // Steering controller

      if(fabs(waypoints[way_state % 4].x - current_pos.x) < 0.1f && fabs(waypoints[way_state % 4].x - current_pos.x))
      {
         way_state = (way_state + 1) % 4;
      }

      float x0,y0,x1,y1,x2,y2;
      x0 = current_pos.x;
      y0 = current_pos.y;
      x1 = waypoints[way_state - 1].x;
      y1 = waypoints[way_state - 1].y;
      x2 = waypoints[way_state % 4].x;
      y2 = waypoints[way_state % 4].y;
      float cross_track_err = fabs((x2-x2)*(y1-y0) - (x1-x0)*(y2-y1))/sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
      float steer_angle = atan(ks*cross_track_err / vector2dMagnitude(vel)); 
      steer_angle /= 0.394;

      // publish results
      cmd_vel.angular.z = steer_angle;
      cmd_vel_pub.publish(cmd_vel);

      loop_rate.sleep();
   }

   return 0;
}

