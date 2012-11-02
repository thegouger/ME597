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

   return 0;
}
