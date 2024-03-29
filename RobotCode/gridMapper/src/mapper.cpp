#include <fstream> 
#include <string>
#include <vector>
#include <list>
#include <queue>
#include <cfloat>

#include "consts.hpp"
#include "mapper.hpp"

#ifdef USE_ROS
   #include <sensor_msgs/LaserScan.h>
#endif

using namespace std;

/* --- Mapping --- */

LaserScanner::LaserScanner(){
   RMax = 1;
   RMin = 0;
   AngMax = PI/2;
   AngMin = -PI/2;
   AngRes = PI/180;

   Beta = 0.01;//0.010;  // rad
   Alpha = 0.16; // .1;   // m <--- wait whatttT??? this should set?!?!

   grid = NULL ;
}

LaserScanner::LaserScanner(OccupancyGrid * Grid){
   LaserScanner();
   grid = Grid ;
}

#ifdef USE_ROS
void LaserScanner::callback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    ranges = msg->ranges;
    AngMax = msg->angle_max;
    AngMin = msg->angle_min;
    RMax = msg->range_max;
    RMin = msg->range_min;
    AngRes = msg->angle_increment;
    if (grid != NULL) updateMap();
}
#endif

OccupancyGrid::OccupancyGrid(float map_x1,float map_x2,float map_y1,float map_y2,float x_res,float y_res) {
   PLow = 0.3;
   P0 = 0.5;
   PHigh = 0.7;
   LPLow = LOGIT(PLow);
   LP0 = LOGIT(P0);
   LPHigh = LOGIT(PHigh);

   x1 = map_x1;
   x2 = map_x2;
   y1 = map_y1;
   y2 = map_y2;
   xRes = x_res;
   yRes = y_res;

   m = (int)((x2-x1)/xRes);
   n = (int)((y2-y1)/yRes); 

   Map = new float*[m];
   for(int i=0; i<m; i++){
      Map[i] = new float[n];
   }

   for (int i=0; i<m; i++) {
       for (int j=0; j<n; j++) {
            Map[i][j] = LP0;
       }
   }
}

float OccupancyGrid::itox (const int i) {
   return x1 + i*xRes + xRes/2.0;
}

int OccupancyGrid::xtoi (const float x) {
   float i = (x-x1-xRes/2.0)/xRes;
   if ( fabs(i-ceil(i)) < fabs(i-floor(i)) )
      return (int)ceil(i);
   return (int)floor(i);
}

float OccupancyGrid::jtoy (const int j) {
   return y1 + j*yRes + yRes/2.0;
}

int OccupancyGrid::ytoj (const float y) {
   float j = (y-y1-yRes/2.0)/yRes;
   if ( fabs(j-ceil(j)) < fabs(j-floor(j)) )
      return (int)ceil(j);
   return (int)floor(j);
}

void OccupancyGrid::loadMap(std::string fileName) {
   std::ifstream infile;
   infile.open("MapLoad");
    for (int i=0; i<m; i++) {
        for (int j=1; j<n; j++) {
            infile >> Map[i][j];
            Map[i][j] = LOGIT(Map[i][j]);
        }
    }
}

void OccupancyGrid::saveMap(std::string fileName) {
   std::ofstream outfile;
   outfile.open("MapSave");
    for (int i=0; i<m; i++) {
        outfile << UNLOGIT(Map[i][0]);
        for (int j=1; j<n; j++) {
            outfile <<" "<< UNLOGIT(Map[i][j]);
        }
        outfile << "\n";
    }
}

void OccupancyGrid::updateCell (int p, int i, int j) {
    if(p<0 && Map[i][j] > -100000 )
    {
      Map[i][j] -= LP0;
      Map[i][j] += LPLow;
    }

    else if(p>0 && Map[i][j] < 100000 )
    {
      Map[i][j] -= LP0;
      Map[i][j] += LPHigh;
    }
}


/* Try to get the index of the measurement at which the angle
   between the laser scan and the cell-robot vector is minimal.
   Assume ranges is structured siuch that k = 0 => meas at AngMin
   Use a boundary of Beta/2 for discarding outlier angles
   */
int LaserScanner::getMinIndex(float phi) {
    if ((phi > AngMax && fabs(phi - AngMax) > Beta / 2) ||
            (phi < AngMin && fabs(phi - AngMin) > Beta/ 2))
        return -1;

    // phi = AngMin + k * AngRes
    int k = floor((phi - AngMin)/AngRes);

    // Check k+1 to see if discretizing pushed us one before
    if (fabs(phi - AngMin - k * AngRes) > fabs(phi - AngMin - (k + 1) * AngRes))
        k = k + 1;

    // Scan AngRes too big, missed this cell angle
    if (fabs(phi - AngMin - k * AngRes) > Beta / 2)
        return -1;

    return k;
}

void LaserScanner::updateMap(void) { // get x,y,theta from ekf message
    int p;
    float r, phi;
    float cx, cy ;
    for (int i=0; i<grid->M(); i++) {
        for (int j=0; j<grid->N(); j++) {
            cx = grid->itox(i);
            cy = grid->jtoy(j);

            // range and phi to current cell
            r = sqrt((pow(cx - x, 2)) + pow(cy - y, 2));

            phi = acos( ((cy-y)*sin(theta) + (cx-x)*cos(theta) )/sqrt(pow(cx-x,2)+pow(cy-y,2)) );
            phi *= ( (cy-y)*cos(theta) - (cx-x)*sin(theta) ) > 0 ? 1 : -1 ; 

            // Most pertinent laser measurement for this cell
            int k = getMinIndex(phi);

            p=0;
            // If outside measured angles
            if (k == -1) {
                p = 0;
            }
            /*else if (r < RMax && ranges[k] < RMin ){
               p = LPLow;
            }*/
            // If outside field of view of the scan range
            else if (r > fmin(RMax, ranges[k]+Alpha/2)) {
                p = 0;
            }
            // If the range measurement was in this cell, likely to be an object
            else if ((ranges[k]< RMax) && (fabs(r-ranges[k])<Alpha/2)) {
                p = 1;
            }
            // If the cell is in front of the range measurement, likely to be empty
            else if (r <= fmin(RMax,ranges[k])) {
            //else if(r <= ranges[k]) {
                p = -1;
            }
            grid->updateCell(p,i,j);
        }
    }
}

void OccupancyGrid::fillMap(float x1, float x2, float y1, float y2) {
   for (int i=xtoi(x1); i<xtoi(x2); i++) {
      for (int j=ytoj(y1); j<ytoj(y2); j++) {
         Map[i][j] = LPHigh;
      }
   }
}

OccupancyGrid::~OccupancyGrid(){
   for (int i=0; i<m; i++) {
      delete Map[i];
   }
   delete Map;
}
/* --- Path Planning --- */
float MAG(float x1,float x2,float y1,float y2) {
   return sqrt ( pow(x2-x1,2) + pow(y2-y1,2) ) ;
}
float MAG(float x,float y) {
   return sqrt ( pow(x,2) + pow(y,2) ) ;
}

float ANG(float x1,float y1, float x2, float y2) {
   float phi;
   phi = acos(x1*x2 + y1*y2 )/(MAG(x1,y1)*MAG(x2,y2));
   phi *= ( y2*x1 - x2*y1 ) > 0 ? 1 : -1 ; 
   return phi;
}

/* Heuristics */
float H(int i, int j, int gi, int gj) {
   return (pow(gi-i,2)+pow(gj-j,2));
}

float StraightLineHeuristic(float x, float y, float gx, float gy) {
   return sqrt(pow(gx-x,2)+pow(gy-y,2));
}

float RobotHeuristic(float x, float y,float theta,float dt, float angRes, float gx, float gy) {
   int i=0;
   float phi = ANG(gx-x,gy-y,cos(theta),sin(theta));
   while (phi > angRes)  { 
      x += cos(theta)*dt;
      y += sin(theta)*dt;
      if (phi < 0) {
         theta += angRes*dt;
      }
      else if (phi > 0 ) {
         theta -= angRes*dt;
      }
      phi = ANG(gx-x,gy-y,cos(theta),sin(theta));
      i++;
   } 

   return dt*i + sqrt(pow(gx-x,2)+pow(gy-y,2));
}
/* End Heuristics */

float OccupancyGrid::proxCost(int i, int j) {
   if ( i < 0 || i >= m) return false;
   if ( j < 0 || j >= n) return false;

   float tmpC,minC = wall_tol;
   for (int a=fmax(0,i-wall_tol); a<fmin(m,i+wall_tol); a++) {
      for (int b=fmax(0,j-wall_tol); b<fmin(n,j+wall_tol); b++) {
         if (Map[a][b] > LP0) {
            tmpC = sqrt( pow(a-i,2) + pow(b-j,2) ) ;
            if (tmpC < minC){
               minC = tmpC;
            }
         }
      }
   }
   if ( minC == wall_tol ) {
      return 0 ;
   }
   return 10000*( wall_tol-minC );
}

bool OccupancyGrid::validPosition(int i, int j) {
   if ( i < 0 || i >= m) return false;
   if ( j < 0 || j >= n) return false;

   if (Map[i][j] > LP0) return false;

   return true ;
}

State * OccupancyGrid::generateNode(int i, int j,int gi, int gj, float g, State * parent) {
   State * S = new State ;
   S->i = i;   S->j = j;
   S->g = g ;
   S->f = g + H(i,j,gi,gj);
   S->parent = parent;
   return S;
}

State * OccupancyGrid::generateNode(float x, float y, float theta, float gx, float  gy, float g, State * parent) {
   State * S = new State ;
   S->x = x;   S->y = y;
   S->theta = theta;
   S->g = g ;
   S->f = g + StraightLineHeuristic(x,y,gx,gy);
   S->parent = parent;
   return S;
}

std::vector<Vector2d> * 
OccupancyGrid::WavePlanner(float sX,float sY,float gX,float gY) {
   int si = xtoi(sX);
   int sj = ytoj(sY);
   int gi = xtoi(gX);
   int gj = ytoj(gY);
   
   State *S; 
   int closed[m][n];
   for (int i=0 ;i<m; i++){
      for (int j=0; j<n; j++) {
         closed[i][j] = 0;
      }
   }

   std::priority_queue<State*, std::vector<State*>, CompareState > pq;
   std::list<State*> freeList;
    
   pq.push(generateNode(si,sj,gi,gj,0,NULL));

   /* Search for the goal Sate */
   int count = 0 ;
   while ( !pq.empty() ) {
      count++;
      if (count > 2*m*n) {
         cout << "WavePlanner failed (node limit reached)\n";
         break;
      }
      S = pq.top();
      pq.pop();
      freeList.push_front(S);

      if (S->i == gi && S->j == gj) break; // reached the goal state

      if ( closed[S->i][S->j] ) continue;
      closed[S->i][S->j] = 1 ;

      /* Add Neighbours */
      if (validPosition(S->i+1,S->j)) {
         if ( closed[S->i+1][S->j] == 0 ) {
            //closed[S->i+1][S->j] = 2 ;
            pq.push(generateNode(S->i+1,S->j,gi,gj,S->g+1+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i-1,S->j)) {
         if ( closed[S->i-1][S->j] == 0 ) {
            //closed[S->i-1][S->j] = 2;
            pq.push(generateNode(S->i-1,S->j,gi,gj,S->g+1+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i,S->j+1)) {
         if ( closed[S->i][S->j+1] == 0 ) {
            //closed[S->i][S->j+1] = 2;
            pq.push(generateNode(S->i,S->j+1,gi,gj,S->g+1+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i,S->j-1)) {
         if ( closed[S->i][S->j-1] == 0 ) {
            //closed[S->i][S->j-1] = 2;
            pq.push(generateNode(S->i,S->j-1,gi,gj,S->g+1+proxCost(S->i,S->j),S));
         }
      }
      /* Diagonal
      if (validPosition(S->i-1,S->j+1)) {
         if ( closed[S->i-1][S->j+1] == 0 ) {
            //closed[S->i-1][S->j-1] = 2;
            pq.push(generateNode(S->i-1,S->j+1,gi,gj,S->g+sqrt(2)+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i+1,S->j+1)) {
         if ( closed[S->i+1][S->j+1] == 0 ) {
            //closed[S->i+1][S->j+1] = 2;
            pq.push(generateNode(S->i+1,S->j+1,gi,gj,S->g+sqrt(2)+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i-1,S->j-1)) {
         if ( closed[S->i-1][S->j-1] == 0 ) {
            //closed[S->i-1][S->j-1] = 2;
            pq.push(generateNode(S->i-1,S->j-1,gi,gj,S->g+sqrt(2)+proxCost(S->i,S->j),S));
         }
      }
      if (validPosition(S->i+1,S->j-1)) {
         if ( closed[S->i+1][S->j-1] == 0 ) {
            //closed[S->i+1][S->j-1] = 2;
            pq.push(generateNode(S->i+1,S->j-1,gi,gj,S->g+sqrt(2)+proxCost(S->i,S->j),S));
         }
      }
      //*/
   }

   /* Form the Path */
   std::vector<Vector2d> *path = new std::vector<Vector2d>;
   if (S != NULL && S->i == gi && S->j == gj) {
      while (S != NULL) {
         Vector2d p; 
         p.x = itox(S->i);
         p.y = jtoy(S->j);
         path->insert(path->begin(),p);
         S = S->parent;
      }
   }

   cout << "Nodes: "<<count << ", Length: " << path->size() << endl;

   /* Free all the memory */
   while ( !pq.empty() ) {
      S = pq.top();
      pq.pop();
      delete S;
   }
   while ( !freeList.empty() ) {
      S = freeList.front();
      freeList.pop_front();
      delete S;
   }

   /* Return the Path */
   return path;
}

void MotionStep (float x, float y, float t, float &x2, float &y2, float &t2, float dt, float w) {
   x2 = x + cos(t)*dt;
   y2 = y + sin(t)*dt;
   t2 = t + w*dt;
}

std::vector<Vector2d> * 
OccupancyGrid::DirectPlanner(float sX,float sY,float Theta,float gX,float gY, float time_step, float turn_res, float turn_count,float goal_tol) {
   State *S; 
   float tol = goal_tol;
   float tx,ty,tang;
   float dt = time_step;
   float w = turn_res;
   float angRes = turn_count;

   std::priority_queue<State*, std::vector<State*>, CompareState > pq;
   std::list<State*> freeList;
   std::list<float> thetaList[m][n];

   int closed[m][n];
   for (int i=0 ;i<m; i++){
      for (int j=0; j<n; j++) {
         closed[i][j] = 0;
      }
   }
    
   pq.push(generateNode(sX,sY,Theta,gX,gY,0,NULL));

   /* Search for the goal Sate */
   int count = 0 ;
   while ( !pq.empty() ) {
      if (count > 5*m*n) {
         std::cout << "DirectPlanner failed (node limit reached)\n" ;
         break;
      }
      S = pq.top();
      pq.pop();

      freeList.push_front(S);

      if ( MAG(S->x,gX,S->y,gY) < tol ) {
         break; // reached the goal state
      }

      if ( closed[xtoi(S->x)][ytoj(S->y)] ) {
         list<float>::iterator it;
         bool alreadyVisited = false ;
         for ( it=thetaList[xtoi(S->x)][ytoj(S->y)].begin() ; it != thetaList[xtoi(S->x)][ytoj(S->y)].end(); it++ ) {
            if ( (*it) == S->theta ) {
               alreadyVisited = true;
               break;
            }
         }
         if ( alreadyVisited ) {
            continue;
         }
      }
      closed[xtoi(S->x)][ytoj(S->y)] = 1 ;
      thetaList[xtoi(S->x)][ytoj(S->y)].push_front(S->theta);

      /* Add Neighbours */
      MotionStep(S->x,S->y,S->theta,tx,ty,tang,dt,0);
      if ( validPosition(xtoi(tx),ytoj(ty)) ) {
         if ( 1 || !closed[xtoi(tx)][ytoj(ty)] ) { 
            //closed[xtoi(tx)][ytoj(ty)] = 1 ; 
            pq.push( generateNode(tx,ty,tang,gX,gY,S->g+MAG(tx,S->x,ty,S->y)+proxCost(xtoi(tx),ytoj(ty)),S));
         }
      }
      for (int i=1; i <=angRes; i++) {
         for (int j=0; j<3; j+=2) {
            MotionStep(S->x,S->y,S->theta,tx,ty,tang,dt,w*i*(-1+j));
            if ( validPosition(xtoi(tx),ytoj(ty)) ) {
               if ( 1 || !closed[xtoi(tx)][ytoj(ty)] ) {
                  //closed[xtoi(tx)][ytoj(ty)] = 1 ; 
                  pq.push( generateNode(tx,ty,tang,gX,gY,S->g+MAG(tx,S->x,ty,S->y)+proxCost(xtoi(tx),ytoj(ty)),S));
               }
            }
         }
      }
      /* End Add Neighbours */
      count++;
   }

   /* Form the Path */
   std::vector<Vector2d> *path = new std::vector<Vector2d>;
   if (MAG(S->x,gX,S->y,gY) < tol ) {
      while (S != NULL) {
         Vector2d p; 
         p.x = S->x;
         p.y = S->y;
         path->insert(path->begin(),p);
         S = S->parent;
      }
   }
   cout << "Nodes: "<<count << ", Length: " << path->size() << endl;

   /* Free all the memory */
   while ( !pq.empty() ) {
      S = pq.top();
      pq.pop();
      delete S;
   }
   while ( !freeList.empty() ) {
      S = freeList.front();
      freeList.pop_front();
      delete S;
   }

   /* Return the Path */
   return path;
}
