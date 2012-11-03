#include <math.h>

// Ensure that map's res is high enough so that discretization is trivial
#define Map_X_Resolution 0.05 // meters
#define Map_Y_Resolution 0.05 // meters

#define LOGIT(p)		 log(p/(1-p))
#define UNLOGIT(p)		 (exp(p)/(1+exp(p)))
#define PI 				 3.1415

struct vector2d
{
    float x;
    float y;
};

/* Corners that define our mapping boundaries */
const vector2d LEFT_BOT_CORNER = {-100, -100};
const vector2d RIGHT_TOP_CORNER  = {100, 100};

const int M = (RIGHT_TOP_CORNER.x - LEFT_BOT_CORNER.x)/Map_X_Resolution;
const int N = (RIGHT_TOP_CORNER.y - LEFT_BOT_CORNER.y)/Map_X_Resolution;

float Map[M][N];

/* LIDAR Parameters */ // <-- These need to be updated to actual values
const float RMax = 1;
const float RMin = 0;
const float AngMax = PI/2;
const float AngMin = PI/2;
const float AngRes = PI/180;

const float Beta = 0.05;  // degrees
const float Alpha = .1;   // m

// Var to put the Range data in
// float ranges[int((AngMax - AngMin)/AngRes)]

/* Set Probibilites */
const float P0 = 0.5;
const float LP0 = LOGIT(P0);

const float PHigh = 0.7;
const float LPHigh = LOGIT(PHigh);

const float PLow = 0.3;
const float LPLow = LOGIT(PLow);

/* node infos */
float x,y,theta;


void initialiseMap() {
    for (int i=0; i<M; i++) {
        for (int j=0; j<N; j++) {
            Map[i][j] = LP0;
        }
    }
}

void updateCell (float p,i,j) {
    Map[i][j] = p + Map[i][j] - LP0;
}

/* Try to get the index of the measurement at which the angle
   between the laser scan and the cell-robot vector is minimal.
   Assume meas_r is structured siuch that k = 0 => meas at AngMin
   Use a boundary of Beta/2 for discarding outlier angles
   */
void getMinIndex(float phi) {
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
    float r, phi, x, y, theta;
    for (int i=0; i<M; i++) {
        for (int j=0; j<N; j++) {
            // Assume origin of the grid at lower bot corner
            x = x - LOWER_BOT_CORNER.x;
            y = y - LOWER_BOT_COERNER.y;
            
            // range and phi to current cell
            r = sqrt((pow(i*Map_X_Resolution-x,2))+pow(j*Map_Y_Resolution-y,2));
            phi = (atan2(j*Map_Y_Resolution-y,i*Map_X_Resolution-x)-theta+PI) % 2*PI - PI;

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

int main() {

    do_ROS_Stuff ()

        initialiseMap();

    while (ROSS::OK()){
        //everytime you get a new lidar measurment,
        // get current position

        updateMap()

            // maybe publish the Map somewhere
            // so others can use it too!
    }

    return 0 ;
}

