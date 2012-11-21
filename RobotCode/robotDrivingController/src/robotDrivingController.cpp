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
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

#define ENCODERS 1
#define USE_SIM
#define LAB2

#ifdef USE_SIM
#include <nav_msgs/Odometry.h>
#endif

using namespace Eigen;

std::ofstream outfile;
const float MAX_OUTPUT_SPEED = 2.0f; //1.0 for actual robot

const float L = 0.283; //0.237; for actual robot

// desired waypoints
Vector2f waypoints[4]; // 4

// Measurements
Vector3f Y_noiseless;
Vector3f Y;
Matrix3f Q( (Matrix3f() << 0.05*0.05, 0,      0,
                              0,      0.05*0.05,  0,
                              0,      0,           0.0524*0.0524).finished() );
Matrix3f R( (Matrix3f() << 0.01*0.01, 0,      0,
                              0,     0.01*0.01,  0,
                              0,     0,          0.1*0.1).finished() );
float current_velocity;
float steer_angle;

// normal random generation
boost::mt19937 rng;
boost::normal_distribution<double> pos(0.0, 0.05);
boost::normal_distribution<double> theta(0.0, 0.0524);
boost::variate_generator<boost::mt19937&,
                         boost::normal_distribution<double> > pos_norm(rng, pos);

boost::variate_generator<boost::mt19937&,
                         boost::normal_distribution<double> > theta_norm(rng, theta);
// EKF params
Eigen::Vector3f mu;
Eigen::Vector3f mu_p;
Eigen::Matrix3f G;
Eigen::Matrix3f Sigma_p;
Eigen::Matrix3f Sigma( Matrix3f().Identity() );
Eigen::Matrix3f K;
Eigen::Matrix3f C( Matrix3f().Identity() );

bool   position_acquired = false; // used to determine whether the IPS has given us a position
double old_time;

#ifdef USE_SIM
void stateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   static float old_x=0, old_y=0;
   double dt = ros::Time::now().toSec() - old_time;
   old_time = ros::Time::now().toSec();

   if(dt > 1)
     return;

   float x,y,Theta;
   x = msg->pose.pose.position.x;
   y = msg->pose.pose.position.y;

   // simulator gives us a quaternion
   float q_x = msg->pose.pose.orientation.x, q_y = msg->pose.pose.orientation.y, q_z = msg->pose.pose.orientation.z, q_w = msg->pose.pose.orientation.w;
 
   Theta = atan2(2*q_w*q_z + 2*q_x*q_y, 1 - 2*q_y*q_y - 2*q_z*q_z);

   if(!position_acquired)
   {
      mu(0) = x;
      mu(1) = y;
      mu(2) = Theta;
      std::cout<<"MU: ******************************************************" << mu;
   }

   current_velocity = sqrt((x-old_x)*(x-old_x) + (y-old_y)*(y-old_y))/dt;

   Y(0) = x;
   Y(1) = y;
   Y(2) = Theta;

   old_x = x;
   old_y = y;
   
   if(!position_acquired)
   {
      position_acquired = true;
      return; // don't proceed with EKF because we don't have a dt yet
   }


   Vector3f epsilon;
   epsilon(0) = pos_norm(); 
   epsilon(1) = pos_norm();
   epsilon(2) = theta_norm();
   
   Y += epsilon;

   // Prediction update
   mu_p(0) = mu(0) + current_velocity*cos(mu(2))*dt;
   mu_p(1) = mu(1) + current_velocity*sin(mu(2))*dt;
   mu_p(2) = mu(2) + (current_velocity*tan(steer_angle)*dt)/L;
   if(mu_p(2) > 3.14159)       mu_p(2) -= 2*3.14159;
   else if(mu_p(2) < -3.14159) mu_p(2) += 2*3.14159; 

   G = (Matrix3f() << 1, 0, -current_velocity*sin(mu(2))*dt,
                     0, 1, current_velocity*cos(mu(2))*dt,
                     0, 0, 1).finished();

   Sigma_p = G*Sigma*(G.transpose()) + R;

   // measurement update
   //K     = Sigma_p*(C.transpose())*((C*Sigma_p*(C.transpose()) + Q).inverse());
   K = Sigma_p*(Sigma_p + Q).inverse();
   mu    = mu_p + ((Y(0) < 900) ? K*(Y - C*mu_p) : (Vector3f() << 0,0,0).finished());
   Sigma = (Matrix3f().Identity() - K*C)*Sigma_p;

   outfile << dt  << " " << x << " " << y << " " << Theta << " " << mu(0) << " " << mu(1) << " "
           << mu(2) << " " << Y(0) << " " << Y(1) << " " << Y(2) << "\n"; 

   // mu(0) = Y(0);
   // mu(1) = Y(1);
   // mu(2) = Y(2);


}
#endif

void indoorPosCallback(const indoor_pos::ips_msg::ConstPtr& msg)
{
   if(!position_acquired) // set mu to initial state
   {
      mu(0) = msg->X;
      mu(1) = msg->Y;
      mu(2) = msg->Yaw/180.0*3.14159;
   }

   // record time between callbacks to accurately determine velocity
   double dt = ros::Time::now().toSec() - old_time;
   old_time = ros::Time::now().toSec();

   // Measurements
   Y(0) = msg->X;
   Y(1) = msg->Y;
   Y(2) = msg->Yaw/180.0*3.14159;

   Y_noiseless = Y;

   if(!position_acquired)
   {
      position_acquired = true;
      return; // don't proceed with EKF because we don't have a dt yet
   }

   Vector3f epsilon;
   epsilon(0) = pos_norm(); 
   epsilon(1) = pos_norm();
   epsilon(2) = theta_norm();

   Y += epsilon;

   // Prediction update
   mu_p(0) = mu(0) + current_velocity*cos(mu(2))*dt;
   mu_p(1) = mu(1) + current_velocity*sin(mu(2))*dt;
   mu_p(2) = mu(2) + (current_velocity*tan(steer_angle)*dt)/L;
   if(mu_p(2) > 3.14159)       mu_p(2) -= 2*3.14159;
   else if(mu_p(2) < -3.14159) mu_p(2) += 2*3.14159; 

   G = (Matrix3f() << 1, 0, -current_velocity*sin(mu(2))*dt,
                     0, 1, current_velocity*cos(mu(2))*dt,
                     0, 0, 1).finished();

   Sigma_p = G*Sigma*(G.transpose()) + R;

   // measurement update
    //K     = Sigma_p*(C.transpose())*((C*Sigma_p*(C.transpose()) + Q).inverse());
   K = Sigma_p*(Sigma_p + Q).inverse();
   mu    = mu_p + ((Y(0) < 900) ? K*(Y - C*mu_p) : (Vector3f() << 0,0,0).finished());
   Sigma = (Matrix3f().Identity() - K*C)*Sigma_p;

   outfile << dt  << " " << msg->X << " " << msg->Y << " " << msg->Yaw/180.0*3.14159 << " " << mu(0) << " " << mu(1) << " "
           << mu(2) << " " << Y(0) << " " << Y(1) << " " << Y(2) << "\n"; 

}

void waypointCallback (const geometry_msgs::Twist::ConstPtr& msg) {
   if(msg->linear.x != waypoints[1](0) && msg->linear.y != waypoints[1](1))
   {
     waypoints[0](0) = waypoints[1](0);//mu(0);
     waypoints[0](1) = waypoints[1](1);//mu(1);
   }

   //waypoints[0](0) = mu(0);
   //waypoints[0](1) = mu(1);
   waypoints[1](0) = msg->linear.x;
   waypoints[1](1) = msg->linear.y;
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

   outfile.open("EKF_test.txt");

   bool enable_steering = false;  

   // desired travel velocity
   double desired_velocity = 0.200; // vel in m/s f

   #ifdef LAB1
   waypoints[0](0) = -1.01f; waypoints[0](1) = -.57f;
   waypoints[1](0) = 1.46f;  waypoints[1](1) = -.57f;
   waypoints[2](0) = 1.46f;  waypoints[2](1) = 1.45f;
   waypoints[3](0) = -1.0f;  waypoints[3](1) = 1.45f;
   int way_state = 1;

   outfile << "Desired velocity: " << desired_velocity << "\n";
   outfile << "Waypoints: \n";
   for (int i=0; i<4; i++) {
      outfile << waypoints[i](0) << "," << waypoints[i](1) << "\n";
   }
   #endif

   // PI tuning consts
   float kp = 100.0f, ki = 10.0f;

   // Stanley constant
   double ks = 2.00; //  0.5f;

   // PID, steering intermediaries
   double err_sum = 0.0f;

   double max_steering_angle = 1.04 * .8; // < 20 deg ? (average of two links)

   // msgs to send/receive
   geometry_msgs::Twist cmd_vel;
   geometry_msgs::Twist estimate;

   // subscribe to odom, publish cmd_vel
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, indoorPosCallback);
   ros::Publisher cmd_vel_pub = nodeHandle.advertise<geometry_msgs::Twist>("cmd_vel", 1);
   ros::Publisher estimate_pub = nodeHandle.advertise<geometry_msgs::Twist>("estimate", 1);

   #ifdef USE_SIM
   ros::Subscriber state_sub = nodeHandle.subscribe("base_pose_ground_truth", 1, stateCallback);
   #endif
   #ifdef LAB2
   ros::Subscriber waypoint_sub = nodeHandle.subscribe("waypoint",1,waypointCallback);
   #endif
   #ifdef ENCODERS
   ros::Subscriber encoder_sub = nodeHandle.subscribe("/data/encoders", 1, encoderCallback);
   #endif

   ros::Rate loop_rate(20); // be sure to use sim time if simulating

   // file output
   // outfile<< "x       y       velocity      desired_vel     vel_output      heading    crosstrack_error   steer_angle\n";

   outfile << "timestamp   actual_x   actual_y     actual_t    velocity    x_estimate    y_estimate   t_estimate   x_mes  y_mes  t_mes \n";
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

    Vector2f heading_vector(cos(mu(2)), sin(mu(2)));

    #ifdef ENCODERS
    float velocity_sign = current_velocity > 0 ? 1.0 : -1.0;
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
#ifdef LAB1
    if( (Vector2f() << waypoints[way_state % 4](0) - mu(0), waypoints[way_state % 4](1) - mu(1)).finished().norm() < 0.5)
    {
       //outfile << "Acheived waypoint: " << way_state << "\n";
       ROS_INFO("Acheived waypoint: %d", way_state);
       way_state = way_state % 4 + 1;
    }
#endif
    float x0,y0,x1,y1,x2,y2;
    x0 = mu(0);
    y0 = mu(1);
    x1 = waypoints[0](0);//waypoints[way_state - 1](0);
    y1 = waypoints[0](1);//waypoints[way_state - 1](1);
    x2 = waypoints[1](0);//waypoints[way_state % 4](0);
    y2 = waypoints[1](1);//waypoints[way_state % 4](1);

    Vector2f path_vector(x2-x1, y2-y1);
    float sign = heading_vector(0)*path_vector(1) - heading_vector(1)*path_vector(0);
    sign /= fabs(sign);
    float heading = sign*acos((path_vector.dot(heading_vector))/path_vector.norm());
    float cross_track_err = ((x2-x1)*(y1-y0) - (x1-x0)*(y2-y1))/path_vector.norm();
      
    steer_angle = heading + atan(ks*cross_track_err/current_velocity); // this is a global so it can be accessed by the EKF
      
    if(steer_angle > max_steering_angle) steer_angle = max_steering_angle;
    else if(steer_angle < -max_steering_angle) steer_angle = -max_steering_angle;

    float steer_angle_normalized = steer_angle/max_steering_angle * 100.0f;  // percentage

    if(steer_angle_normalized > 100.0f)       steer_angle_normalized = 100.0f;
    else if(steer_angle_normalized < -100.0f) steer_angle_normalized = -100.0f;

    ROS_INFO("Current x: %f, Current y: %f, Cross track error: %f, Heading: %f, CT Heading: %f, Steering:%f, Current velocity: %f\n", x0, y0, cross_track_err, heading, atan(ks*cross_track_err/current_velocity), steer_angle_normalized/100.0f, current_velocity);
      
    cmd_vel.angular.z = steer_angle_normalized;

    cmd_vel.linear.x  = 1.5f; // /= 100.0f;
    // publish results
    #ifdef LAB2
    if (waypoints[1](1) == 1337 && waypoints[1](0) == 1337 ) {
       cmd_vel.linear.x = 0 ;
       cmd_vel.angular.z = 0.0f; // steer_angle_normalized;
    }
    #endif
    #ifdef USE_SIM
    cmd_vel.angular.z /= 100.0f;
    #endif
    cmd_vel_pub.publish(cmd_vel);

    estimate.linear.x = mu(0);
    estimate.linear.y = mu(1);
    estimate.angular.z = mu(2);

    // publish estimates
   //  if(Y_noiseless(0) < 900)
   //  {
   //    estimate.linear.x = Y_noiseless(0);
   //    estimate.linear.y = Y_noiseless(1);
   //    estimate.angular.z = Y_noiseless(2);
   //  }

    estimate_pub.publish(estimate);

    loop_rate.sleep();
   }

   return 0;
}

