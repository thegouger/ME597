#include <iostream>
#include <math.h>
#include <list>
#include <vector>
#include <functional>

#include "consts.hpp"

#ifdef USE_SFML
   #include <SFML/Graphics.hpp>
   #include "body.hpp"
#endif
#ifdef USE_ROS
   #include <ros/ros.h>
   #include <indoor_pos/ips_msg.h>
   #include <geometry_msgs/Twist.h>
   #include <sensor_msgs/LaserScan.h>
   #include <std_msgs/Float32MultiArray.h>
#endif
#ifdef USE_SIMULATOR
   #include <nav_msgs/Odometry.h>
#endif

using namespace std;

#include "mapper.hpp"
//#include "planner.hpp"

/* All The Colours */
#ifdef USE_SFML
sf::Color RobotColor(214,246,0);
sf::Color GhostColor(9,101,205);
sf::Color PathColor(255,0,192);

sf::Color Border(32,32,32);
sf::Color BG(106,106,67);
sf::Color Empty(86,105,106);
sf::Color Full(126,30,32);
sf::Color Unknown(32,32,32);
/* ^^ Colours ^^ */
#endif

LaserScanner scanner;

/* control flags */
bool update_map;

/* node infos */
float x,y,theta;
float mu_x,mu_y,mu_theta;

/* Goal Position */
float gx; // m
float gy; // m


#ifdef USE_ROS
void scannerCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { 
   scanner.callback(msg);
}
#ifdef USE_SIMULATOR
void stateCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
   x = msg->pose.pose.position.x;
   y = msg->pose.pose.position.y;
   theta = msg->pose.pose.orientation.z;
   double roll, pitch;

   // simulator gives us a quaternion
   float q_x = msg->pose.pose.orientation.x, q_y = msg->pose.pose.orientation.y, q_z = msg->pose.pose.orientation.z, q_w = msg->pose.pose.orientation.w;
   theta = atan2(2*q_w*q_z + 2*q_x*q_y, 1 - 2*q_y*q_y - 2*q_z*q_z);
   /*
   scanner.x = x;
   scanner.y = y;
   scanner.theta = theta;
   */
}
#else
void IPSCallback(const indoor_pos::ips_msg::ConstPtr& msg) {
   x = msg->X;
   y = msg->Y;
   theta = msg->Yaw;
   scanner.x = x;
   scanner.y = y;
   scanner.theta = theta;
}
#endif
void ekfCallback(const geometry_msgs::Twist::ConstPtr& msg) { 
   mu_x = msg->linear.x;
   mu_y = msg->linear.y;
   mu_theta = msg->angular.z;
   scanner.x = mu_x;
   scanner.y = mu_y;
   scanner.theta = mu_theta;
}
#endif

/* --- Graphics Related Functions --- */
#ifdef USE_SFML
void  colorTransform (float p,sf::Color & C) {
   float P0 = 0.5;
   float PHigh = 0.7;
   float PLow = 0.3; // <- will fix later
   if (p > PHigh) {
      C = Full;
   }
   else if (p > P0) {
      C.r = (Full.r-Unknown.r)/(PHigh-P0)*(p-P0)+Unknown.r;
      C.g = (Full.g-Unknown.g)/(PHigh-P0)*(p-P0)+Unknown.g;
      C.b = (Full.b-Unknown.b)/(PHigh-P0)*(p-P0)+Unknown.b;
   }
   else if (p == P0) {
      C = Unknown;
   }
   else if (p < P0) {
      C.r = -(Empty.r-Unknown.r)/(P0-PLow)*(p-PLow)+Empty.r;
      C.g = -(Empty.g-Unknown.g)/(P0-PLow)*(p-PLow)+Empty.g;
      C.b = -(Empty.b-Unknown.b)/(P0-PLow)*(p-PLow)+Empty.b;
   }
   else {
      C = Empty; 
   }
}

/* Here is where we draw the map */
void drawMap(OccupancyGrid *grid,sf::RenderWindow * W) {
   sf::Shape Cell;
   sf::Color C; 
   W->Draw(sf::Shape::Rectangle(X1-WT,Y1-WT,X2+WT,Y2+WT,Border));
   for (int i=0; i<grid->M(); i++) {
      for (int j=0; j<grid->N(); j++) {
         colorTransform(grid->cellProbability(i,j),C);
         Cell = sf::Shape::Rectangle (X1+i*XPPC, Y2-j*YPPC, X1+(i+1)*XPPC, Y2-YPPC*(j+1),C); 
         W->Draw(Cell);
      }
   }
}

   //*
void drawPath(const vector<Vector2d> * path,const sf::Color &C,sf::RenderWindow *W) {
   float x1,x2,y1,y2;
   sf::Shape Line;
   for (int i=1; i<path->size(); i++) {
      x1 = (path->at(i-1).x);
      y1 = (path->at(i-1).y);
      x2 = (path->at(i).x);
      y2 = (path->at(i).y);
      Line = sf::Shape::Line(X1+(x1-Map_X1)*PPM, Y2-(y1-Map_Y1)*PPM, X1+(x2-Map_X1)*PPM, Y2-(y2-Map_Y1)*PPM,PathThickness,C);
      W->Draw(Line);
   }
}

#endif

/* --- Main Function --- */
int main (int argc, char* argv[]) {
   #ifdef USE_SFML
   sf::RenderWindow Window(sf::VideoMode(XRES,YRES,32),NAME) ;
   #ifdef LIMITFPS
      Window.SetFramerateLimit(MAXFPS) ;
   #endif


   /* ----- Setup ----- */
   sf::Shape Goal,Start;

   Transform Robot,Ghost,Head,GHead;
   Head.Rect(-RL/10,-RW/4,RL/10,RW/4,Unknown);
   Head.SetCenter(RL/10,0);
   Head.SetLPosition(RL-RCX,0);
   Robot.Rect(0,0,RL,RW,RobotColor,1,Unknown);
   Robot.SetCenter(RCX,RCY);
   Robot.AddChild(&Head);

   GHead.Rect(-RL/10,-RW/4,RL/10,RW/4,Unknown);
   GHead.SetCenter(RL/10,0);
   GHead.SetLPosition(RL-RCX,0);
   Ghost.Rect(0,0,RL,RW,GhostColor,1,Unknown);
   Ghost.SetCenter(RCX,RCY);
   Ghost.AddChild(&GHead);
   #endif

   x = y = theta = 0 ;
   mu_x = mu_y = mu_theta = 0 ;
   gx = gy = 0;
   bool plan = false; 

   OccupancyGrid Grid(Map_X1,Map_X2,Map_Y1,Map_Y2,Map_XRes,Map_YRes);
   //*/ Test Obsticals
   Grid.fillMap(0.55,0.8,0.25,1);   
   Grid.fillMap(-0.5,-0.25,-0.5,0.5);   
   Grid.fillMap(0.55,0.8,-1,-0.25);   
   Grid.fillMap(1.5,2,-0.5,0.5);   
   //*/
   vector<Vector2d> truePath,ekfPath;;
   Vector2d tmpP;

   #ifdef USE_ROS
   ros::init(argc, argv, "gridMapper");
   ROS_INFO("Combined Mapper and Path Planner");

   ros::NodeHandle nodeHandle;

   scanner.grid = &Grid;

   geometry_msgs::Twist WayPoint ;
   ros::Publisher waypoint_pub = nodeHandle.advertise<geometry_msgs::Twist>("waypoint", 1);

   ros::Subscriber scanner_sub = nodeHandle.subscribe("base_scan/scan", 1 ,scannerCallback);
   ros::Subscriber ekf_sub = nodeHandle.subscribe("estimate",1,ekfCallback);
   #ifdef USE_SIMULATOR
   ros::Subscriber state_sub = nodeHandle.subscribe("base_pose_ground_truth", 1, stateCallback); 
   #else
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, IPSCallback);
   #endif
   #endif

   /* ------------------------ */
#ifdef USE_ROS
   while(ros::ok()) {// <-- Replace with ROS 
      ros::spinOnce();
#else
   while(1) {
#endif 
      /* ------ Loop ------ */
      #ifdef USE_SFML
      Window.Clear(BG) ;
      drawMap(&Grid,&Window);
      #endif

      /* Test Code
      x = sin(theta);
      y = cos(theta);
      theta += 0.1 ;
      mu_theta = theta - 0.3;
      mu_x = 0.2+sin(mu_theta);
      mu_y = -0.2+cos(mu_theta);
      /* End Test Code */
      
      /* Planning */
      WayPoint.linear.x = 1337;
      WayPoint.linear.y = 1337;
      if (plan) {
         vector<Vector2d> * path;
         
         Window.Draw(Start);
         Window.Draw(Goal);

         tmpP.x = mu_x;
         tmpP.y = mu_y ;
         ekfPath.insert(ekfPath.end(),tmpP);
         tmpP.x = x;
         tmpP.y = y ;
         truePath.insert(truePath.end(),tmpP);
         
         drawPath(&ekfPath,GhostColor,&Window);
         drawPath(&truePath,RobotColor,&Window);

         /* Wavefront */
         #if PATH_PLANNER<1 || PATH_PLANNER >1
         path = Grid.WavePlanner(mu_x,mu_y,gx,gy);
         drawPath(path,Unknown,&Window);
         if ( path->size() > 3 ) {
            WayPoint.linear.x = path->at(3).x;
            WayPoint.linear.y = path->at(3).y;
         }
         delete path;
         #endif
         
         /* A* */
         #if PATH_PLANNER>0
         path = Grid.findPath2(mu_x,mu_y,mu_theta,gx,gy);
         drawPath(path,PathColor,&Window);
         if ( path->size() > 2 ) {
            WayPoint.linear.x = path->at(2).x;
            WayPoint.linear.y = path->at(2).y;
         }
         delete path;
         #endif
      }
      #ifdef USE_ROS
      waypoint_pub.publish(WayPoint);
      #endif
      /* End Planning */

      #ifdef USE_SFML
      Ghost.SetGPosition(X1+PPM*(mu_x-Map_X1),Y2-PPM*(mu_y-Map_Y1));
      Ghost.SetGRotation(mu_theta*180/PI);
      Robot.SetGPosition(X1+PPM*(x-Map_X1),Y2-PPM*(y-Map_Y1));
      Robot.SetGRotation(theta*180/PI);

      Ghost.Draw(&Window);
      Robot.Draw(&Window);
      /* ------------------------ */
      Window.Display() ;
      sf::Event Event;
      while (Window.GetEvent(Event)) {
         if (Event.Type == sf::Event::Closed) {
             Window.Close();
         }   
         if (Event.Type == sf::Event::MouseButtonPressed) {
            if (Event.MouseButton.Button == sf::Mouse::Right) {
               const sf::Input& Input = Window.GetInput();
               float MouseX = Input.GetMouseX();
               float MouseY = Input.GetMouseY();
               gx = (MouseX-X1)/PPM + Map_X1;
               gy = (Y2-MouseY)/PPM + Map_Y1;

               Goal = sf::Shape::Circle(X1+PPM*(gx-Map_X1),Y2-PPM*(gy-Map_Y1), goal_tol*PPM, PathColor,2,Unknown) ;
            }
         }
         if (Event.Type == sf::Event::KeyPressed) {
            if(Event.Key.Code == sf::Key::A) {
               plan = !plan;
               Start = sf::Shape::Circle(X1+PPM*(x-Map_X1),Y2-PPM*(y-Map_Y1), goal_tol*PPM, BG,2,Unknown);
               ekfPath.clear();
               truePath.clear();
            }
            if(Event.Key.Code == sf::Key::P) {
               update_map = !update_map;
            }
            if(Event.Key.Code == sf::Key::S) {
               Grid.saveMap("Map");
            }
            if(Event.Key.Code == sf::Key::L) {
               Grid.loadMap("Map");
            }
         }
      }   
      #endif
   }
   #ifdef USE_SFML
   return EXIT_SUCCESS ; 
   #else 
   return 0;
   #endif
}
