#include <ros/ros.h>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <indoor_pos/ips_msg.h>
#include <clearpath_horizon/Encoder.h>
#include <clearpath_horizon/Encoders.h>
#include <fstream>

#define ENCODERS 1


std::ofstream outfile;
const float MAX_OUTPUT_SPEED = 100.0f;

struct vector2d
{
   float x;
   float y;
};

float vector2dMagnitude(vector2d v) { return sqrt(v.x*v.x + v.y*v.y); }

// state globals
vector2d current_pos = {0.0f, 0.0f};
vector2d old_pos = {0.0f, 0.0f};
vector2d current_vel = {0.0f, 0.0f};
vector2d old_vel = {0.0f, 0.0f};
vector2d delta_vel = {0.0f, 0.0f};

float    current_yaw;

bool     position_acquired; // used to determine whether the IPS has given us a position
double   old_time;

void indoorPosCallback(const indoor_pos::ips_msg::ConstPtr& msg)
{
   // record time between callbacks to accurately determine velocity
   double timeBetweenCallbacks = ros::Time::now().toSec() - old_time;
   old_time = ros::Time::now().toSec();

   current_pos.x = msg->X;
   current_pos.y = msg->Y;
   current_yaw   = msg->Yaw/180.0*3.14159;

   #ifndef ENCODERS

   if(position_acquired)
   {
      current_vel.x = (current_pos.x - old_pos.x) / timeBetweenCallbacks;
      current_vel.y = (current_pos.y - old_pos.y) / timeBetweenCallbacks;
      delta_vel.x = (current_vel.x - old_vel.x) / timeBetweenCallbacks;
      delta_vel.y = (current_vel.y - old_vel.y) / timeBetweenCallbacks;
   }

   old_pos.x = current_pos.x;
   old_pos.y = current_pos.y;
   old_vel.x = current_vel.x;
   old_vel.y = current_vel.y;

   #endif

   position_acquired = true;
}

#ifdef ENCODERS
void encoderCallback(const clearpath_horizon::EncodersConstPtr& msg)
{
   current_vel.x = msg->encoders[0].speed;
   position_acquired = true;
}
#endif

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "robotDrivingController");
   ROS_INFO("Combined velocity and steering controller");

   ros::NodeHandle nodeHandle;

   outfile.open("square_track_PID7_more_tolerance.txt");

   bool enable_steering = false;  
   bool execute_once = true;

   // desired travel velocity
   double desired_velocity = 0.500; // vel in m/s f

   // desired waypoints
   vector2d waypoints[4]; // 4
   waypoints[0].x = -1.01f; waypoints[0].y = -.57f;
   waypoints[1].x = 1.46f;    waypoints[1].y = -.57f;
   waypoints[2].x = 1.46f;  waypoints[2].y = 1.45f;
   waypoints[3].x = -1.0f;   waypoints[3].y = 1.45f;
   int way_state = 1;

   outfile << "Desired velocity: " << desired_velocity << "\n";
   outfile << "Waypoints: \n";
   for (int i=0; i<4; i++) {
      outfile << waypoints[i].x << "," << waypoints[i].y << "\n";
   }

   // PID tuning consts
   float kp = 100.0f, kd = 0.0f, ki = 10.0f;

   // Stanley constant
   double ks = 1.0; //  0.5f;
   // nodeHandle.getParam("stanley_const", ks);

   // PID, steering intermediaries
   double err_sum = 0.0f;

   double max_steering_angle = 0.394; // 20 deg ?
   // nodeHandle.getParam("max_steering_angle", max_steering_angle);

   // msgs to send/receive
   geometry_msgs::Twist cmd_vel;

   // subscribe to odom, publish cmd_vel
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, indoorPosCallback);
   ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);

   #ifdef ENCODERS
   ros::Subscriber encoder_sub = nodeHandle.subscribe("/data/encoders", 1, encoderCallback);
   #endif

   ros::Rate loop_rate(20); // be sure to use sim time if simulating

   // file output
   outfile<< "x       y       velocity      desired_vel     vel_output      heading    crosstrack_error   steer_angle\n";

   while(ros::ok())
   {
      ros::spinOnce();

      // haven't gotten a callback yet
      if(!position_acquired) // current_pos.x == 0.0)
      {
	 loop_rate.sleep();
	 continue;
      }

      // PID controller

      vector2d heading_vector = {cos(current_yaw), sin(current_yaw)};
      float velocity_sign = (heading_vector.x*current_vel.x + heading_vector.y*current_vel.y);
      velocity_sign = velocity_sign > 0 ? 1.0 : -1.0;

      #ifdef ENCODERS
      velocity_sign = current_vel.x > 0 ? 1.0 : -1.0;
      ROS_INFO("V sign: %f", velocity_sign);
      #endif

      // PID terms
      float error = desired_velocity - velocity_sign*vector2dMagnitude(current_vel);
      err_sum += error;

      float dv = (vector2dMagnitude(current_vel) - vector2dMagnitude(old_vel));

      float vel_output = error * kp + dv * kd + err_sum * ki;

      // limit velocity output
      if(vel_output > MAX_OUTPUT_SPEED)       vel_output = MAX_OUTPUT_SPEED;
      else if(vel_output < -100) vel_output = -100.0; // don't drive backwards

      cmd_vel.linear.x = vel_output;

      // Steering controller
      if(fabs(waypoints[way_state % 4].x - current_pos.x) < 0.30 && fabs(waypoints[way_state % 4].y - current_pos.y) < 0.30f)
      {
         outfile << "Acheived waypoint: " << way_state << "\n";
         ROS_INFO("Acheived waypoint: %d", way_state);
         way_state = way_state % 4 + 1;
      }

      if(sqrt(pow(waypoints[way_state % 4].x - current_pos.x, 2.0) + pow(waypoints[way_state % 4].y - current_pos.y, 2.0) < 0.4))
      {
         outfile << "Acheived waypoint: "  << way_state << "\n";
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

      vector2d path_vector    = {x2-x1, y2-y1};
      float heading = acos((path_vector.x * heading_vector.x + path_vector.y * heading_vector.y)/vector2dMagnitude(path_vector)); //atan2((y2-y1), (x2-x1)) - current_yaw;
      float cross_track_err = ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
      float steer_angle = heading + atan(ks*cross_track_err/vector2dMagnitude(current_vel)); 
      steer_angle = steer_angle/max_steering_angle * 100.0f;  // percentage

      if(steer_angle > 100.0f)       steer_angle = 100.0f;
      else if(steer_angle < -100.0f) steer_angle = -100.0f;

      ROS_INFO("Current x: %f, Current y: %f, Cross track error: %f, Heading: %f, Steering:%f, Velocity: %f\n", x0, y0, cross_track_err, heading, steer_angle, vel_output);
      
      outfile<< current_pos.x << " " <<  current_pos.y << " " << vector2dMagnitude(current_vel) << " "<< desired_velocity << " " << vel_output << " " << heading << " " << cross_track_err << " " << steer_angle << "\n";

      // ROS_INFO("Current x: %f, Current y: %f, Error:  %f, Sum: %f \n", x0, y0, error, err_sum);

      cmd_vel.angular.z = steer_angle;

      // publish results
      cmd_vel_pub.publish(cmd_vel);

      loop_rate.sleep();
   }

   return 0;
}

