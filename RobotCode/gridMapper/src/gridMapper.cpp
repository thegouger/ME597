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

using namespace std;

#include "mapper.hpp"
//#include "planner.hpp"

/* All The Colours */
#ifdef USE_SFML
sf::Color RobotColor(214,246,0);
sf::Color PathColor(255,0,192);

sf::Color Border(32,32,32);
sf::Color BG(106,106,67);
sf::Color Empty(86,105,106);
sf::Color Full(126,30,32);
sf::Color Unknown(32,32,32);
#endif
/* ^^ Colours ^^ */

LaserScanner scanner;

/* control flags */
bool update_map;

/* node infos */
float x,y,theta;

#ifdef USE_ROS
void scannerCallback(const sensor_msgs::LaserScan::ConstPtr& msg) { 
   scanner.callback(msg);
}
void stateCallback(const geometry_msgs::Twist::ConstPtr& msg) { 
   x = msg->linear.x;
   y = msg->linear.y;
   theta = msg->angular.z;
   scanner.x = x;
   scanner.y = y;
   scanner.theta = theta;
}
void IPSCallback(const indoor_pos::ips_msg::ConstPtr& msg) {
   x = msg->X;
   y = msg->Y;
   theta = msg->Yaw;
   scanner.x = x;
   scanner.y = y;
   scanner.theta = theta;
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
void drawPath(OccupancyGrid *grid,vector<Vector2d> * path, sf::RenderWindow *W) {
   float x1,x2,y1,y2;
   sf::Shape Line;
   for (int i=1; i<path->size(); i++) {
      x1 = grid->xtoi(path->at(i-1).x);
      y1 = grid->ytoj(path->at(i-1).y);
      x2 = grid->xtoi(path->at(i).x);
      y2 = grid->ytoj(path->at(i).y);
      Line = sf::Shape::Line(X1+x1*XPPC, Y2-y1*YPPC, X1+x2*XPPC, Y2-YPPC*y2,PathThickness,PathColor);
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
   Transform Robot,Head;
   Head.Rect(-RL/10,-RW/4,RL/10,RW/4,Unknown);
   Head.SetCenter(RL/10,0);
   Head.SetLPosition(RL-RCX,0);
   Robot.Rect(0,0,RL,RW,RobotColor);
   Robot.SetCenter(RCX,RCY);
   Robot.AddChild(&Head);
   #endif
   //fillMap(0.5,0.5,1,1);   
   x = y = theta = 0 ;
   bool plan = false;; 
   OccupancyGrid Grid(Map_X1,Map_X2,Map_Y1,Map_Y2,Map_XRes,Map_YRes);

   #ifdef USE_ROS
   ros::init(argc, argv, "gridMapper");
   ROS_INFO("Combined Mapper and Path Planner");

   ros::NodeHandle nodeHandle;

   scanner.grid = &Grid;

   std_msgs::Float32MultiArray Path ;

   //ros::Subscriber scanner_sub = nodeHandle.subscribe("scan", 1 ,scannerCallback);
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, IPSCallback);
   //ros::Subscriber state_sub = nodeHandle.subscribe("estimate",1,stateCallback);
   ros::Publisher path_pub = nodeHandle.advertise<std_msgs::Float32MultiArray>("path", 1);
   #endif

   Grid.fillMap(0.55,0.8,0.25,1);   
   Grid.fillMap(-0.5,-0.25,-0.5,0.5);   
   Grid.fillMap(0.55,0.8,-1,-0.25);   

   /* ------------------------ */
#ifdef USE_ROS
   while(ros::ok()) {// <-- Replace with ROS 
      //ros::spinOnce();
#else
   while(1) {
#endif 
      /* ------ Loop ------ */
      #ifdef USE_SFML
      Window.Clear(BG) ;
      drawMap(&Grid,&Window);
      #endif
      
      //*
      plan = false;
      if (plan) {
         //vector<Vector2d> * path = Grid.findPath(x,y,0,0);
         vector<Vector2d> * path = Grid.findPath2(x,y,theta,1.8,0);
         if (path == NULL) {
            plan = false;
         }
         else {
            drawPath(&Grid,path,&Window);
            delete path;
         }
      }
      //*/
      #ifdef USE_ROS
      //path_pub.publish(Path);
      #endif

      /* ~~ Test Code ~~ */
      float v = .1;
      x += v*cos(theta);
      y += v*sin(theta);
      theta += 10*PI/180.0;
      /* ~~ End Test ~~ */

      #ifdef USE_SFML
      Robot.SetGPosition(X1+PPM*(x-Map_X1),Y2-PPM*(y-Map_Y1));
      Robot.SetGRotation(theta*180/PI);
      Robot.Draw(&Window);
      /* ------------------------ */
      Window.Display() ;
      sf::Event Event;
      while (Window.GetEvent(Event)) {
         if (Event.Type == sf::Event::Closed) {
             Window.Close();
         }   
         if (Event.Type == sf::Event::KeyPressed) {
            if(Event.Key.Code == sf::Key::A) {
               plan = !plan;
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
