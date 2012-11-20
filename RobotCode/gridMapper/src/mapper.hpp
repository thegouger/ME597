#ifndef __MAPPER_HPP__
#define __MAPPER_HPP__

#include <iostream>
#include <cmath>
#include <vector>
#include <string>
#ifdef USE_ROS
   #include <ros/ros.h>
   #include <sensor_msgs/LaserScan.h>
#endif

#define LOGIT(p) log(p/(1-p))
#define UNLOGIT(p) (exp(p)/(1+exp(p)))
#define PI 3.14159

 
struct Vector2d {
   float x;
   float y;
};

class State {
   public:
   State() {};
   int i;
   int j;
   float x;
   float y;
   float theta;
   float g;
   float f;
   State * parent;
};

struct CompareState : public std::binary_function<State*, State*, bool> {
   bool operator()(const State* s1, const State* s2) const {
      return s1->f > s2->f ;
   }
};

class OccupancyGrid {
   public:
      OccupancyGrid(const float map_x1,const float map_x2,const float map_y1,const float map_y2,const float x_res,const float y_res);

      float ** Map;
      
      int M() { return m; }
      int N() { return n; }
      
      float cellProbability(int i,int j) { return UNLOGIT(Map[i][j]); }

      float itox (const int i) ;
      int xtoi (const float x) ;
      float jtoy (const int j) ;
      int ytoj (const float y) ;

      void saveMap(std::string fileName) ;
      void loadMap(std::string fileName) ;
      
      void fillMap(float x1, float y1, float x2, float y2) ;

      void updateCell (int p,int i,int j) ;


      std::vector<Vector2d> * WavePlanner(float sX, float sY, float gX,float gY) ;
      std::vector<Vector2d> * findPath(float sX, float sY, float gX,float gY) ;
      std::vector<Vector2d> * findPath2(float sX, float sY, float Theta, float gX,float gY) ;

      bool validPosition(int i, int j) ;
      ~OccupancyGrid();
   private:
      int m,n;

      float PLow ;
      float P0 ;
      float PHigh ;

      float LPLow ;
      float LP0 ;

      float LPHigh;
      float x1,x2,y1,y2;
      float xRes,yRes;

      State * generateNode(int i, int j,int gi, int gj, float g, State * parent) ;
      State * generateNode(float x, float y, float theta, float gx, float  gy, float g, State * parent) ;
};

class LaserScanner {
   public:
      LaserScanner();
      LaserScanner(OccupancyGrid * );

      OccupancyGrid * grid;
      float x,y,theta;

#ifdef USE_ROS
      void callback (const sensor_msgs::LaserScan::ConstPtr& msg);
#endif

      ~LaserScanner() {}
      private:
         float RMax ;
         float RMin ;
         float AngMax ;
         float AngMin ;
         float AngRes ;

         float Beta ;  // degrees
         float Alpha ;   // m
         std::vector<float> ranges;

         void updateMap() ;
         int getMinIndex(float phi) ;
};

#endif
