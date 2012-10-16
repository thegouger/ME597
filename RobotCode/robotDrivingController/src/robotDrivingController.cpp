#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <indoor_pos/ips_msg.h>

const float MAX_OUTPUT_SPEED = 100.0f;

struct vector2d
{
   float x;
   float y;
};

float vector2dMagnitude(vector2d v) { return sqrt(v.x*v.x + v.y*v.y); }

vector2d current_pos;
float    current_yaw;

void indoorPosCallback(const indoor_pos::ips_msg::ConstPtr& msg)
{
   current_pos.x = msg->X;
   current_pos.y = msg->Y;
   current_yaw   = msg->Yaw;
}

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "robotDrivingController");
   ROS_INFO("Combined velocity and steering controller");

   ros::NodeHandle nodeHandle;

   bool enable_steering;
   nodeHandle.getParam("enable_steering", enable_steering);

   // desired travel velocity
   double desired_velocity; // 0.5f
   nodeHandle.getParam("desired_velocity", desired_velocity);

   // desired waypoints
   vector2d waypoints[4]; 
   waypoints[0].x = 5.0f; waypoints[0].y = -5.0f;
   waypoints[1].x = 10.0f; waypoints[1].y = -5.0f;
   waypoints[2].x = 10.0f; waypoints[2].y = 0.0f;
   waypoints[3].x = 5.0f; waypoints[3].y = 0.0f;
   int way_state = 1;

   // PID tuning consts
   float kp = 10.0f, kd = 0.0f, ki = 0.0f;

   // Stanley constant
   double ks; //  0.5f;
   nodeHandle.getParam("stanley_const", ks);

   // PID, steering intermediaries
   vector2d old_pos = {0.0f, 0.0f}, old_vel = {0.0f, 0.0f};
   double err_sum = 0.0f;

   double max_steering_angle; // 20 deg ?
   nodeHandle.getParam("max_steering_angle", max_steering_angle);

   // msgs to send/receive
   geometry_msgs::Twist cmd_vel;

   // subscribe to odom, publish cmd_vel
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, indoorPosCallback);
   ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   ros::Rate loop_rate(20); // be sure to use sim time if simulating
   while(ros::ok())
   {
      ros::spinOnce();

      // PID controller

      // current velocity in units/sec
      vector2d vel;
      vel.x = (current_pos.x - old_pos.x) * 20.0f;
      vel.y = (current_pos.y - old_pos.y) * 20.0f;

      // PID terms
      float error = desired_velocity - vector2dMagnitude(vel);
      err_sum += error;
      float dv = (vector2dMagnitude(vel) - vector2dMagnitude(old_vel))*20.0f;

      float vel_output = error * kp + dv * kd + err_sum * ki;

      // limit velocity output
      if(vel_output > MAX_OUTPUT_SPEED)       vel_output = MAX_OUTPUT_SPEED;
      else if(vel_output < -MAX_OUTPUT_SPEED) vel_output = -MAX_OUTPUT_SPEED;

      cmd_vel.linear.x = vel_output;

      // Steering controller
      if(fabs(waypoints[way_state % 4].x - current_pos.x) < 0.1f && fabs(waypoints[way_state % 4].y - current_pos.y) < 0.1f)
      {
         ROS_INFO("Acheived waypoint: %d", way_state);
         way_state = way_state % 4 + 1;
      }

      float x0,y0,x1,y1,x2,y2;
      x0 = current_pos.x;
      y0 = current_pos.y;
      x1 = waypoints[way_state - 1].x;
      y1 = waypoints[way_state - 1].y;
      x2 = waypoints[way_state % 4].x;
      y2 = waypoints[way_state % 4].y;

      float heading = atan2((y2-y1), (x2-x1)) - current_yaw;
      float cross_track_err = ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
      float steer_angle = heading + atan(ks*cross_track_err/vector2dMagnitude(vel)); 
      steer_angle = steer_angle/max_steering_angle * 100.0f;  // percentage

      if(steer_angle > 100.0f)       steer_angle = 100.0f;
      else if(steer_angle < -100.0f) steer_angle = - 100.0f;

      //ROS_INFO("Current x: %f, Current y: %f, Cross track error: %f, Heading: %f, Steering:%f", x0, y0, cross_track_err, heading, steer_angle);
      //ROS_INFO("Waypoint: %f,%f", x1, y1);

      if(enable_steering)
        cmd_vel.angular.z = steer_angle;

      // publish results
      cmd_vel_pub.publish(cmd_vel);

      // set old values
      old_pos.x = current_pos.x;
      old_pos.y = current_pos.y;
      old_vel.x = vel.x;
      old_vel.y = vel.y;

      loop_rate.sleep();
   }

   return 0;
}

