#include <fstream> 
#include <string>

#include "consts.hpp"
#include "mapper.hpp"
#include <sensor_msgs/LaserScan.h>

using namespace std;

/* --- Mapping --- */

LaserScanner::LaserScanner(){
   RMax = 1;
   RMin = 0;
   AngMax = PI/2;
   AngMin = -PI/2;
   AngRes = PI/180;

   Beta = 0.05;  // degrees
   Alpha = .1;   // m <--- wait whatttT??? this should set?!?!

   grid = NULL ;
}

LaserScanner::LaserScanner(OccupencyGrid * Grid){
   OccupencyGrid();
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

OccupencyGrid::OccupencyGrid(float map_x1,float map_x2,float map_y1,float map_y2,float x_res,float y_res) {
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

   m = (x2-x1)/xRes;
   n = (y2-y1)/yRes; 

   *Map = new float[m];
   for(int i=0; i<m; i++){
      Map[i] = new float[n];
   }

   for (int i=0; i<m; i++) {
       for (int j=0; j<n; j++) {
            Map[i][j] = LP0;
           //Map[i][j] = PLow+i*(PHigh-PLow)/M;
       }
   }
}

float OccupencyGrid::itox (const int i) {
   return x1 + i*xRes + xRes/2.0;
}

int OccupencyGrid::xtoi (const float x) {
   float i = (x-x1-xRes/2.0)/xRes;
   if ( fabs(i-ceil(i)) < fabs(i-floor(i)) )
      return (int)ceil(i);
   return (int)floor(i);
}

float OccupencyGrid::jtoy (const int j) {
   return y1 + j*yRes + yRes/2.0;
}

int OccupencyGrid::ytoj (const float y) {
   float j = (y-y1-yRes/2.0)/yRes;
   if ( fabs(j-ceil(j)) < fabs(j-floor(j)) )
      return (int)ceil(j);
   return (int)floor(j);
}

void OccupencyGrid::loadMap(std::string fileName) {
   ifstream infile;
   infile.open("MapLoad");
    for (int i=0; i<m; i++) {
        for (int j=1; j<n; j++) {
            infile >> Map[i][j];
            Map[i][j] = LOGIT(Map[i][j]);
        }
    }
}

void OccupencyGrid::saveMap(std::string fileName) {
   ofstream outfile;
   outfile.open("MapSave");
    for (int i=0; i<m; i++) {
        outfile << UNLOGIT(Map[i][0]);
        for (int j=1; j<n; j++) {
            outfile <<" "<< UNLOGIT(Map[i][j]);
        }
        outfile << endl;
    }
}

void OccupencyGrid::updateCell (int p,int i,int j) {
    if (Map[i][j] < 5 && Map[i][j] > -5) { // <-- update with less stupid numbers
         Map[i][j] -= LP0;
         Map[i][j] += p<0 ? LPLow : (p>0 ? LPHigh : LP0 ) ;
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

            phi = atan2(cy - y, cx - x) - theta  ;
            phi = fmod(atan2(cy - y, cx - x) - theta+PI,2*PI)-PI;
            phi = acos( ((cy-y)*sin(theta) + (cx-x)*cos(theta))/sqrt(pow(cx-x,2)+pow(cy-y,2)) );
            phi *= (cy-y)*cos(theta) - (cx-x)*sin(theta) > 0 ? 1 : -1 ; 

            if (fabs(phi) > 2*PI) {
            ROS_INFO("phi=%f, theta=%f",phi,theta);
            }

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
                p = -1;
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
   for (int i=0; i<m; i++) {
      delete Map[i];
   }
   delete Map;
}
