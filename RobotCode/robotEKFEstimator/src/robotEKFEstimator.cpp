// ROS
#include <ros/ros.h>
// ROS msgs
#include <geometry_msgs/Twist.h>
#include <indoor_pos/ips_msg.h>
#include <clearpath_horizon/Encoder.h>
#include <clearpath_horizon/Encoders.h>
// Other
#include <iostream>
#include <fstream>
#include <cmath>
#include <Eigen/Dense>

// globals
Eigen::Vector3f Y; // measurements

// generate normally distributed random numbers based on box-muller transform
// or use std::tr1?

using namespace std;

int main()
{
   Eigen::MatrixXd m(2,2);
   
   m(0,0) = 3;
   m(1,0) = 2.5;
   m(0,1) = -1;
   m(1,1) = 5;

   cout << "M: " << m << endl;
   cout << "M transpose: " << m.transpose() << endl;

   // actual code

   // EKF params
   Eigen::Vector3f mu_p;
   Eigen::Vector3f mu; // init with initial measurements
   Eigen::Matrix   G(3,3);
   Eigen::Matrix   Sigma_p(3,3);
   Eigen::Matrix   Sigma(3,3);
   Eigen::Matrix   Sigma_prev(3,3);
   Eigen::Matrix   K(3,3);
   Eigen::Matrix   C(3,3);
   C = C.Identity();

   // every loop iteration
   // prediction update
   mup(1) = mu(1) + v*cos(mu(3))*dt;
   mup(2) = mu(2) + v*sin(mu(3))*dt;
   mup(3) = mu(3) + (v*tan(steer)*dt)/L;
   
   G(1,1) = 1; G(1,2) = 0; G(1,3) = -v*sin(mu(3));
   G(2,1) = 0; G(2,2) = 1; G(2,3) = v*cos(mu(3));
   G(3,1) = 0; G(3,2) = 0; G(3,3) = 1;

   Sigma_p = G*Sigma_prev*(G.transpose()) + R;
   
   // measurement update
   K     = Sigma_p*(C.transpose)*((C*Sigma_p*(C.transpose()) + Q).inverse());
   mu    = mu_p + K*(Y - C*mu_p);
   Sigma = (Sigma.identity() - K*C)*Sigma_p;
   
   return 0;
}
