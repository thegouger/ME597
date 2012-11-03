// ros related libraries
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <indoor_pos/ips_msg.h>
#include <clearpath_horizon/Encoder.h>
#include <clearpath_horizon/Encoders.h>

// others
#include <fstream>
#include <cmath>
#include <Eigen/Dense>
#include <random>

#define ENCODERS 1

using namespace Eigen;

std::ofstream outfile;
const float MAX_OUTPUT_SPEED = 100.0f;

// Measurements
Vector3f Y;
const Matrix3f Q( (Matrix3f() << 0.05*0.05, 0,      0,
                                0,      0.05*0.05,  0,
                                0,      0           0.0524*0.0524).finished() );
float current_velocity;
float steer_angle;

// EKF params
Eigen::Vector3f mu;
Eigen::Vector3f mu_p;
Eigen::Matrix3f G;
Eigen::Matrix3f Sigma_p;
Eigen::Matrix3f Sigma;
Eigen::Matrix3f K;
Eigen::Matrix3f C( Matrix3f().Identity() );

bool   position_acquired; // used to determine whether the IPS has given us a position
double old_time;

void indoorPosCallback(const indoor_pos::ips_msg::ConstPtr& msg)
{
   if(!position_acquired) // set mu to initial state
   {
      mu(1) = msg->X;
      mu(2) = msg->Y;
      mu(3) = msg->Yaw/180.0*3.14159;
   }

   // record time between callbacks to accurately determine velocity
   double timeBetweenCallbacks = ros::Time::now().toSec() - old_time;
   old_time = ros::Time::now().toSec();

   // Measurements
   Y(1) = msg->X;
   Y(2) = msg->Y;
   Y(3) = msg->Yaw/180.0*3.14159;

   if(!position_acquired)
   {
      position_acquired = true;
      return; // don't proceed with EKF because we don't have a dt yet
   }

   Vector3f epsilon;
   epsilon(1) = std::tr1::normal_distribution<double> normal(0.0f, sqrt(Q(1,1)));
   epsilon(2) = std::tr1::normal_distribution<double> normal(0.0f, sqrt(Q(2,2,));
   epsilon(3) = std::tr1::normal_distribution<double> normal(0.0f, sqrt(Q(3,3,));

   Y += epsilon;

   // Prediction update
   mup(1) = mu(1) + v*cos(mu(3))*dt;
   mup(2) = mu(2) + v*sin(mu(3))*dt;
   mup(3) = mu(3) + (v*tan(steer)*dt)/L;

   G(1,1) = 1; G(1,2) = 0; G(1,3) = -v*sin(mu(3));
   G(2,1) = 0; G(2,2) = 1; G(2,3) = v*cos(mu(3));
   G(3,1) = 0; G(3,2) = 0; G(3,3) = 1;

   Sigma_p = G*Sigma*(G.transpose()) + R;

   // measurement update
   K     = Sigma_p*(C.transpose)*((C*Sigma_p*(C.transpose()) + Q).inverse());
   mu    = mu_p + K*(Y - C*mu_p);
   Sigma = (Matrix3f().identity() - K*C)*Sigma_p;

}

#ifdef ENCODERS
void encoderCallback(const clearpath_horizon::EncodersConstPtr& msg)
{
   current_velocity = msg->encoders[0].speed;
}
#endif

int main(int argc, char* argv[])
{

   ros::init(argc, argv, "robotDrivingController");
   ROS_INFO("Combined velocity and steering controller");

   ros::NodeHandle nodeHandle;

   outfile.open("test.txt");

   bool enable_steering = false;  
   bool execute_once = true;

   // desired travel velocity
   double desired_velocity = 0.200; // vel in m/s f

   // desired waypoints
   Vector2f waypoints[4]; // 4
   waypoints[0](1) = -1.01f; waypoints[0](2) = -.57f;
   waypoints[1](1) = 1.46f;  waypoints[1](2) = -.57f;
   waypoints[2](1) = 1.46f;  waypoints[2](2) = 1.45f;
   waypoints[3](1) = -1.0f;  waypoints[3](2) = 1.45f;
   int way_state = 1;

   outfile << "Desired velocity: " << desired_velocity << "\n";
   outfile << "Waypoints: \n";
   for (int i=0; i<4; i++) {
      outfile << waypoints[i](1) << "," << waypoints[i](2) << "\n";
   }

   // PID tuning consts
   float kp = 100.0f, kd = 0.0f, ki = 10.0f;

   // Stanley constant
   double ks = 1.0; //  0.5f;

   // PID, steering intermediaries
   double err_sum = 0.0f;

   double max_steering_angle = 0.394; // 20 deg ?

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

      Vector2f heading_vector(cos(current_yaw), sin(current_yaw));

      #ifdef ENCODERS
      velocity_sign = current_velocity > 0 ? 1.0 : -1.0;
      ROS_INFO("V sign: %f", velocity_sign);
      #endif

      // PID terms
      float error = desired_velocity - velocity_sign*current_velocity;
      err_sum += error;

      float vel_output = error * kp + err_sum * ki;

      // limit velocity output
      if(vel_output > MAX_OUTPUT_SPEED)       vel_output = MAX_OUTPUT_SPEED;
      else if(vel_output < -100) vel_output = -100.0;

      cmd_vel.linear.x = vel_output;

      // Steering controller
      if( (Vector2f() << waypoints[way_state % 4](1) - mu(1), waypoints[way_state % 4](2) - mu(2)).norm() < 0.3)
      {
         outfile << "Acheived waypoint: " << way_state << "\n";
         ROS_INFO("Acheived waypoint: %d", way_state);
         way_state = way_state % 4 + 1;
      }

      float x0,y0,x1,y1,x2,y2;
      x0 = mu(1);
      y0 = mu(2);
      x1 = waypoints[way_state - 1](1);
      y1 = waypoints[way_state - 1](2);
      x2 = waypoints[way_state % 4](1);
      y2 = waypoints[way_state % 4](2);

      Vector2f path_vector(x2-x1, y2-y1);
      float heading = acos((path_vector.dot(heading_vector))/path_vector.norm());
      float cross_track_err = ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/path_vector.norm();
      
      steer_angle = heading + atan(ks*cross_track_err/current_velocity); // this is a global so it can be accessed by the EKF
      float steer_angle_normalized = steer_angle/max_steering_angle * 100.0f;  // percentage

      if(steer_angle_normalized > 100.0f) steer_angle_normalized = 100.0f;
      else if(steer_angle_normalized < -100.0f)      steer_angle_normalized = -100.0f;

      ROS_INFO("Current x: %f, Current y: %f, Cross track error: %f, Heading: %f, Steering:%f, Velocity: %f\n", x0, y0, cross_track_err, heading, steer_angle, vel_output);
      
      cmd_vel.angular.z = steer_angle_normalized;

      // publish results
      cmd_vel_pub.publish(cmd_vel);

      loop_rate.sleep();
   }

   return 0;
}

