#include <math.h>


/* Map Parameters */
#define Map_Length 10 // x, meters
#define Map_Width 10  // y, meters
#define Map_X_Resolution 0.05 // meters 
#define Map_Y_Resolution 0.05 // meters

const int M = ceil(Map_Length*Map_X_Resolution) ;
const int N = ceil(Map_Width*Map_Y_Resolution) ; 
float map[M][N];

/* LIDAR Parameters */ // <-- These need to be updated to actual values
const float RMax = 1;
const float RMin = 0;
const float Beta = 1;
const float Alpha = 1;
// Var to put the Range data in
// Var to put the Theta Data in

/* Set Probibilites */ 
const float P0 = 0.5;
const float LP0 = logit(P0);

const float PHigh = 0.7;
const float LPHigh = logit(PHigh);

const float PLow = 0.3;
const float LPLow = logit(PLow);

/* node infos */
float x,y,theta; 


float logit(float p) {
   return log(p/(1-p));
}

float unlogit(float p) {
   return exp(p)/(1+exp(p));
}

void initialiseMap() {
   for (i=0, i<M; i++) {
      for (j=0; j<N; j++) {
         map[i][j] = LP0;
      }
   }
}

void updateCell (float p,i,j) {
   map[i][j] = p + map[i][j] - LP0;
}


void updateMap(); // get x,y,theta from ekf message 
   float r,phi;
   for (i=0, i<M; i++) {
      for (j=0; j<N; j++) {
         r = sqrt((pow(i*Map_X_Resolution-x,2))+pow(j*Map_Y_Resolution-y,2));
         phi = (atan2(j*Map_Y_Resolution-y,i*Map_X_Resolution-x)-theta+pi) % 2*pi - pi;

         int k = 0;// ??

         // If out of range, or behind range measurement, or outside of field
         //  of view, no new information is available 
         if (r > fmin(RMax, meas_r[k]+Alpha/2) || (fabs(phi-meas_phi[k])>Beta/2)) {
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

int main() {

   do_ROS_Stuff () 

   initialiseMap();

   while (ROSS::OK()){
      //everytime you get a new lidar measurment,
      // get current position

      updateMap()

      // maybe publish the map somewhere
      // so others can use it too!
   }

   return 0 ; 
}

