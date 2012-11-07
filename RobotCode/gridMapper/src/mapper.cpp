#include <fstream> 

#include "consts.hpp"
#include "mapper.hpp"
#include <sensor_msgs/LaserScan.h>

/* --- Mapping --- */
#define LOGIT(p) log(p/(1-p))
#define UNLOGIT(p) (exp(p)/(1+exp(p)))
#define PI 3.14159

LaserScanner::LaserScanner(){
   RMax = 1;
   RMin = 0;
   AngMax = PI/2;
   AngMin = -PI/2;
   AngRes = PI/180;

   Beta = 0.05;  // degrees
   Alpha = .1;   // m
   numRanges = (int)((AngMax - AngMin)/AngRes);

   grid = NULL ;
}

LaserScanner::LaserScanner(OccupencyGrid * Grid){
   RMax = 1;
   RMin = 0;
   AngMax = PI/2;
   AngMin = -PI/2;
   AngRes = PI/180;

   Beta = 0.05;  // degrees
   Alpha = .1;   // m
   numRanges = (int)((AngMax - AngMin)/AngRes);

   grid = Grid ;
}

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


/* Set Probibilites */
const float P0 = 0.5;
const float LP0 = LOGIT(P0);

const float PHigh = 0.8;
const float LPHigh = LOGIT(PHigh);

const float PLow = 0.3;
const float LPLow = LOGIT(PLow);




OccupencyGrid::OccupencyGrid(float map_x1,float map_x2,float map_y1,float map_y2,float x_res,float y_res) {
   x1 = map_x1;
   x2 = map_x2;
   y1 = map_y1;
   y2 = map_y2;

   M = (y2-y1)/yRes;
   N = (x2-x1)/xRes; 

   Map = new float[M][N];

   for (int i=0; i<M; i++) {
       for (int j=0; j<N; j++) {
            Map[i][j] = LP0;
           //Map[i][j] = PLow+i*(PHigh-PLow)/M;
       }
   }
}

float OccupencyGrid::jtoy (const int i) {
   return y2 - j*yRes + yRes/2.0;
}

float OccupencyGrid::itox (const int i) {
   return x1 + i*xRes + xRes/2.0;
}

void OccupencyGrid::loadMap() {
   ifstream infile;
   infile.open("MapLoad");
   update_map = false; 
    for (int i=0; i<M; i++) {
        for (int j=1; j<N; j++) {
            infile >> Map[i][j];
            Map[i][j] = LOGIT(Map[i][j]);
        }
    }
}

void OccupencyGrid::saveMap() {
   ofstream outfile;
   outfile.open("MapSave");
    for (int i=0; i<M; i++) {
        outfile << UNLOGIT(Map[i][0]);
        for (int j=1; j<N; j++) {
            outfile <<" "<< UNLOGIT(Map[i][j]);
        }
        outfile << endl;
    }
}


void OccupencyGrid::updateCell (float p,int i,int j) {
    if (Map[i][j] < 5 && Map[i][j] > -5) {
         Map[i][j] = p + Map[i][j] - LP0;
    }
}


/* Try to get the index of the measurement at which the angle
   between the laser scan and the cell-robot vector is minimal.
   Assume ranges is structured siuch that k = 0 => meas at AngMin
   Use a boundary of Beta/2 for discarding outlier angles
   */
int getMinIndex(float phi) {
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

void OccupencyGrid::updateMap(void) { // get x,y,theta from ekf message
    float p;
    float r, phi;
    float cx, cy ;
    for (int i=0; i<grid->M; i++) {
        for (int j=0; j<grid->N; j++) {
            cy = Map_BL_y + Map_Y_Resolution*i;
            cx = Map_BL_x + Map_X_Resolution*j;

            // range and phi to current cell
            r = sqrt((pow(cx - x, 2)) + pow(cy - y, 2));
            phi = atan2(cy - y, cx - x) - theta  ;
            phi = fmod(atan2(cy - y, cx - x) - theta+PI,2*PI)-PI;

            phi = acos( ((cy-y)*sin(theta) + (cx-x)*cos(theta))/sqrt(pow(cx-x,2)+pow(cy-y,2)) );
            phi *= (cy-y)*cos(theta) - (cx-x)*sin(theta) > 0 ? 1 : -1 ; 

            if (fabs(phi) > 2*PI) {
            ROS_INFO("phi=%f, theta=%f",phi,theta);
            }
            // Most pertinent laser measurement for this cell
            int k = getMinIndex(phi);

            p=grid->LP0;
            // If outside measured angles
            if (k == -1) {
                p = grid->LP0;
            }
            /*else if (r < RMax && ranges[k] < RMin ){
               p = LPLow;
            }*/
            // If outside field of view of the scan range
            else if (r > fmin(RMax, ranges[k]+Alpha/2)) {
                p = grid->LP0;
            }
            // If the range measurement was in this cell, likely to be an object
            else if ((ranges[k]< RMax) && (fabs(r-ranges[k])<Alpha/2)) {
                p = grid->LPHigh;
            }
            // If the cell is in front of the range measurement, likely to be empty
            else if (r <= fmin(RMax,ranges[k])) {
                p = grid->LPLow;
            }
            grid->updateCell(p,i,j);
        }
    }
}

void OccupencyGrid::fillMap(float x1, float y1, float x2, float y2) {
   
   for (int i=(y1-Map_BL_y)/Map_Y_Resolution; i<(y2-Map_BL_x)/Map_Y_Resolution; i++) {
      for (int j=(x1-Map_BL_x)/Map_X_Resolution; j<(x2-Map_BL_x)/Map_X_Resolution; j++) {
         Map[i][j] = LPHigh;
      }
   }
}

OccupencyGrid::~OccupencyGrid(){
   delete Map[][];
}
