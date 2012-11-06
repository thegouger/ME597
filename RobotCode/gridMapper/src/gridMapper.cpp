#include "consts.hpp"
#include <iostream>
#include <math.h>
#include <list>
#include <queue>
#include <vector>
#include <functional>
#ifdef USE_SFML
   #include <SFML/Graphics.hpp>
   #include "body.hpp"
#endif
#ifdef USE_ROS
   #include <ros/ros.h>
   #include <geometry_msgs/Twist.h>
   #include <sensor_msgs/LaserScan.h>
#endif

using namespace std;

/* All The Colours */
#ifdef USE_SFML
sf::Color RobotColor(214,246,0);
sf::Color PathColor(255,0,192);

sf::Color Border(32,32,32);
sf::Color BG(106,106,67);
sf::Color Empty(86,105,106);
sf::Color Full(126,30,32);
sf::Color Unknown(60,60,60);
#endif
/* ^^ Colours ^^ */

void updateMap();

/* node infos */
float x,y,theta;

/* --- Mapping --- */
#define LOGIT(p) log(p/(1-p))
#define UNLOGIT(p) (exp(p)/(1+exp(p)))
#define PI 3.14159

const int M = Map_Width/Map_Y_Resolution;
const int N = Map_Length/Map_X_Resolution; 
float Map[M][N];

/* LIDAR Parameters */ // <-- These need to be updated to actual values
const float RMax = 1;
const float RMin = 0;
const float AngMax = PI/2;
const float AngMin = -PI/2;
const float AngRes = PI/180;

const float Beta = 0.05;  // degrees
const float Alpha = .1;   // m
const int numRanges = int((AngMax - AngMin)/AngRes);

// Var to put the Range data in
//float ranges[numRanges];

/* Set Probibilites */
const float P0 = 0.5;
const float LP0 = LOGIT(P0);

const float PHigh = 0.7;
const float LPHigh = LOGIT(PHigh);

const float PLow = 0.3;
const float LPLow = LOGIT(PLow);



void initialiseMap() {
    for (int i=0; i<M; i++) {
        for (int j=0; j<N; j++) {
            Map[i][j] = LP0;
            Map[i][j] = PLow+i*(PHigh-PLow)/M;
        }
    }
}

void updateCell (float p,int i,int j) {
    Map[i][j] = p + Map[i][j] - LP0;
}

#ifdef USE_ROS
/*
void laserScannerCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // The number of scans shouldn't vary
    if (msg.ranges.size() != numRanges) {
        ROS_ERROR("Assumed number of scans not the same. Assumed %d, got %d", 
                    numRanges, msg.ranges.size());
    } else {
        for(uint i = 0; i < msg.ranges.size(); i++){
            ranges[i] = msg.ranges[i];
        }
        updateMap();
    }
}
*/
void stateCallback();

#endif

/* Try to get the index of the measurement at which the angle
   between the laser scan and the cell-robot vector is minimal.
   Assume meas_r is structured siuch that k = 0 => meas at AngMin
   Use a boundary of Beta/2 for discarding outlier angles
   */
int getMinIndex(float phi) {
    if ((phi > AngMax && fabs(phi - AngMax) > Beta / 2) ||
            (phi < AngMin && fabs(phi - AngMin) > Beta/ 2))
        return -1;

    // phi = AngMin + k * AngRes
    int k = ceil((phi - AngMin)/AngRes);

    // Check k+1 to see if discretizing pushed us one before
    if (fabs(phi - AngMin - k * AngRes) > fabs(phi - AngMin - (k + 1) * AngRes))
        k = k + 1;

    // Scan AngRes too big, missed this cell angle
    if (fabs(phi - AngMin - k * AngRes) > Beta / 2)
        return -1;

    return k;
}

void updateMap(void) { // get x,y,theta from ekf message
    float p;
    float r, phi;
    float cx, cy ;
    float meas_r[100]; // <--Only here so it will compile
    for (int i=0; i<M; i++) {
        for (int j=0; j<N; j++) {
            cy = Map_BL_y + Map_Y_Resolution*i;
            cx = Map_BL_x + Map_X_Resolution*j;

            // range and phi to current cell
            r = sqrt((pow(cx - x, 2)) + pow(cy - y, 2));
            phi = atan2(cy - y, cx - x) - theta;

            // Most pertinent laser measurement for this cell
            int k = getMinIndex(phi);

            // If outside measured angles
            if (k == -1) {
                p = LP0;
            }
            // If outside field of view of the scan range
            else if (r > fmin(RMax, meas_r[k]+Alpha/2)) {
                p = LP0;
            }
            // If the range measurement was in this cell, likely to be an object
            else if ((meas_r[k]< RMax) && (fabs(r-meas_r[k])<Alpha/2)) {
                p = LPHigh;
            }
            // If the cell is in front of the range measurement, likely to be empty
            else if (r < meas_r[k]) {
                p = LPLow;
            }
            updateCell(p,i,j);
        }
    }
}

void fillMap(float x1, float y1, float x2, float y2) {
   
   for (int i=(y1-Map_BL_y)/Map_Y_Resolution; i<(y2-Map_BL_x)/Map_Y_Resolution; i++) {
      for (int j=(x1-Map_BL_x)/Map_X_Resolution; j<(x2-Map_BL_x)/Map_X_Resolution; j++) {
         Map[i][j] = LPHigh;
      }
   }
}

/* --- Path Planning --- */

/* Manhattan Distance Heuristic */
float H(int i, int j, int gi, int gj) {
   return 0.5 * (pow(gi-i,2)+pow(gj-j,2));
}

struct Vector2d {
   float x;
   float y;
};

class State {
   public:
   State() {};
   bool operator>(const State& right) const;
   bool operator<(const State& right) const;
   int i;
   int j;
   float g;
   float f;
   State * parent;
};

struct CompareState : public std::binary_function<State*, State*, bool> {
   bool operator()(const State* s1, const State* s2) const {
      return s1->f > s2->f ;
   }
};

bool validPosition(int i, int j) {
   if ( i < 0 || i >= M) return false;
   if ( j < 0 || j >= N) return false;
   return true ;
}

State * generateNode(int i, int j,int gi, int gj, float g, State * parent) {
   State * S = new State ;
   S->i = i;   S->j = j;
   g = Map[i][j] > 0.6 ? 100000+g : g;
   S->g = g + Map[i][j];
   S->f = g + H(i,j,gi,gj);
   S->parent = parent;
   return S;
}

vector<Vector2d> * findPath(float goalX,float goalY) {
   int si = (y-Map_BL_y)/Map_Y_Resolution;
   int sj = (x-Map_BL_x)/Map_X_Resolution;
   int gi = (goalY-Map_BL_y)/Map_Y_Resolution;
   int gj = (goalX-Map_BL_x)/Map_X_Resolution;
   
   vector<Vector2d> *path = new vector<Vector2d>;
   State *S; 

   std::priority_queue<State*, vector<State*>, CompareState > pq;
   list<State*> freeList;
    
   pq.push(generateNode(si,sj,gi,gj,0,NULL));

   /* Search for the goal Sate */
   while ( !pq.empty() ) {
      S = pq.top();
      pq.pop();
      freeList.push_front(S);

      if (S->i == gi && S->j == gj) break; // goal state

      /* Add Neighbours */
      if (validPosition(S->i+1,S->j)) {
         pq.push(generateNode(S->i+1,S->j,gi,gj,S->g,S));
      }
      if (validPosition(S->i-1,S->j)) {
         pq.push(generateNode(S->i-1,S->j,gi,gj,S->g,S));
      }
      if (validPosition(S->i,S->j+1)) {
         pq.push(generateNode(S->i,S->j+1,gi,gj,S->g,S));
      }
      if (validPosition(S->i,S->j-1)) {
         pq.push(generateNode(S->i,S->j-1,gi,gj,S->g,S));
      }
   }

   /* Form the Path */
   if (S != NULL && S->i == gi && S->j == gj) {
      while (S != NULL) {
         Vector2d p; 
         p.x = Map_BL_x + Map_X_Resolution*S->j; 
         p.y = Map_BL_y + Map_Y_Resolution*S->i;
         path->insert(path->begin(),p);
         S = S->parent;
      }
   }

   /* Free all the memory */
   while ( !freeList.empty() ) {
      S = freeList.front();
      freeList.pop_front();
      delete S;
   }

   /* Return the Path */
   return path;
}

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
void drawMap(sf::RenderWindow * W) {
   sf::Shape Cell;
   sf::Color C; 
   W->Draw(sf::Shape::Rectangle(X1-WT,Y1-WT,X2+WT,Y2+WT,Border));
   for (int i=0; i<M; i++) {
      for (int j=0; j<N; j++) {
         colorTransform(Map[i][j],C);
         Cell = sf::Shape::Rectangle (X1+i*XPPC, Y1+j*YPPC, X1+(i+1)*XPPC, Y1+YPPC*(j+1),C); 
         W->Draw(Cell);
      }
   }
   
}

void drawPath(vector<Vector2d> * path, sf::RenderWindow *W) {
   float cx, cy;
   sf::Shape Cell;
   for (int i=0; i<path->size(); i++) {
      cy = (path->at(i).x-Map_BL_x)/Map_X_Resolution;
      cx = (path->at(i).y-Map_BL_y)/Map_Y_Resolution;
      Cell = sf::Shape::Rectangle (X1+cx*XPPC, Y1+cy*YPPC, X1+(cx+1)*XPPC, Y1+YPPC*(cy+1),PathColor); 
      W->Draw(Cell);
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

   #ifdef USE_ROS
   ros::init(argc, argv, "gridMapper");
   ROS_INFO("Combined Mapper and Path Planner");

   ros::NodeHandle nodeHandle;

   //ros::Subscriber scanner_sub = nodeHandle.subscribe("scan", 1 , laserScannerCallback);
   //ros::Subscriber state_sub = nodeHandle.subscrube("estimate",1,stateCallback);
   //ros::Publisher path_pub = nodeHandle.advertise<vector<Vector2d> >("path", 1);

   #endif

   /* ----- Setup ----- */
   Transform Robot;
   Robot.Rect(0,0,RW,RH,RobotColor);
   Robot.SetCenter(RCX,RCY);
   #endif
   initialiseMap();
   fillMap(0.5,0.5,1,1);   
   fillMap(-1,-1,-0.5,-0.5);   
   
   float rad = 3.14159 /180 ; 
   int k = 180;
   float amp = 2;
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
      
      y = 0 + amp * sin(k*rad);
      x = 0 + amp * cos(k*rad);
      k++;

      //*
      vector<Vector2d> * path = findPath(0,0);
      //*/
      #ifdef USE_ROS
      //path_pub.publish(*path);
      #endif

      #ifdef USE_SFML
      drawPath(path,&Window);
      Robot.SetGPosition(X1+PPM*(y-Map_BL_y),Y1+PPM*(x-Map_BL_x));
      Robot.SetGRotation(theta);
      Robot.Draw(&Window);
      /* ------------------------ */
      Window.Display() ;
      sf::Event Event;
      while (Window.GetEvent(Event)) {
         if (Event.Type == sf::Event::Closed) {
             Window.Close();
         }   
         if (Event.Type == sf::Event::KeyPressed && Event.Key.Code == sf::Key::A) {
            //?
         }
      }   
      #endif
      delete path;
   }
   #ifdef USE_SFML
   return EXIT_SUCCESS ; 
   #else 
   return 0;
   #endif
}
