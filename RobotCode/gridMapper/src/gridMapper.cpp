#include <iostream>
#include <math.h>
#include <list>
#include <queue>
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
void stateCallback(const geometry_msgs::Twist::ConstPtr& msg) { 
   x = msg->linear.x;
   y = msg->linear.y;
   theta = msg->angular.z;
}
void IPSCallback(const indoor_pos::ips_msg::ConstPtr& msg) {
   x = msg->X;
   y = msg->Y;
   theta = msg->Yaw;
}
#endif

/* --- Graphics Related Functions --- */
#ifdef USE_SFML
void  colorTransform (float p,sf::Color & C) {
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
void drawMap(OccupencyMap *grid,sf::RenderWindow * W) {
   sf::Shape Cell;
   sf::Color C; 
   W->Draw(sf::Shape::Rectangle(X1-WT,Y1-WT,X2+WT,Y2+WT,Border));
   for (int i=0; i<grid->M(); i++) {
      for (int j=0; j<grid->N(); j++) {
         colorTransform(grid->cellProbobility(i,j),C);
         Cell = sf::Shape::Rectangle (X1+i*XPPC, Y1+j*YPPC, X1+(i+1)*XPPC, Y1+YPPC*(j+1),C); 
         W->Draw(Cell);
      }
   }
   
}

void drawPath(vector<Vector2d> * path, sf::RenderWindow *W) {
   /*
   float cx, cy;
   sf::Shape Cell;
   for (int i=0; i<path->size(); i++) {
      cy = path->at(i).x-Map_BL_x;
      cx = path->at(i).y-Map_BL_y;
      Cell = sf::Shape::Rectangle (X1+cx*XPPC, Y1+cy*YPPC, X1+(cx+1)*XPPC, Y1+YPPC*(cy+1),PathColor); 
      W->Draw(Cell);
   }
   */
}
#endif

/* --- Main Function --- */
int main (int argc, char* argv[]) {
   #ifdef USE_SFML
   sf::RenderWindow Window(sf::VideoMode(XRES,YRES,32),NAME) ;
   #ifdef LIMITFPS
      Window.SetFramerateLimit(MAXFPS) ;
   #endif

   #ifdef USE_ROS
   ros::init(argc, argv, "gridMapper");
   ROS_INFO("Combined Mapper and Path Planner");

   ros::NodeHandle nodeHandle;

   OccupencyGrid Grid(Map_X1,Map_X2,Map_Y1,Map_Y2,Map_XRes,Map_YRes);

   std_msgs::Float32MultiArray Path ;

   ros::Subscriber scanner_sub = nodeHandle.subscribe("scan", 1 , LIDAR.callback);
   ros::Subscriber ips_sub    = nodeHandle.subscribe("indoor_pos", 1, IPSCallback);
   //ros::Subscriber state_sub = nodeHandle.subscribe("estimate",1,stateCallback);
   ros::Publisher path_pub = nodeHandle.advertise<std_msgs::Float32MultiArray>("path", 1);

   #endif
   scanner.grid = &Grid;

   /* ----- Setup ----- */
   update_map = true;
   bool plan = false;; 
   Transform Robot,Head;
   Head.Rect(0,0,RH/3,RH/5,Unknown);
   Head.SetCenter(RH/5,RH/5);
   Head.SetLPosition(0,RH/2-RH/10);
   Robot.Rect(0,0,RW,RH,RobotColor);
   Robot.SetCenter(RCX,RCY);
   Robot.AddChild(&Head);
   #endif
   //fillMap(0.5,0.5,1,1);   
   //fillMap(-1,-1,-0.5,-0.5);   

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
      drawMap(&Window);
      #endif
      
      /*
      if (plan) {
         vector<Vector2d> * path = findPath(1,1);
         if (path == NULL) {
            plan = false;
         }
         else {
            drawPath(path,&Window);
            delete path;
         }
      }
      //*/
      #ifdef USE_ROS
      //path_pub.publish(Path);
      scanner.x = x;
      scanner.y = y;
      scanner.theta = theta;
      #endif

      #ifdef USE_SFML
      Robot.SetGPosition(X1+PPM*(x-Map_X1),Y2+PPM*(y-Map_Y1));
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
               saveMap();
            }
            if(Event.Key.Code == sf::Key::L) {
               loadMap();
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
