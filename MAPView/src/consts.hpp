#ifndef _CONSTS_HPP_
#define _CONSTS_HPP_

#define NAME "HIAJLAK"
//*
#define LIMITFPS
#define MAXFPS 100  
//*/

/* You Can Change These Things */

#define PPM 50 // Pixels per Meter
#define WT 20 // Pool Wall Thickness

#define RobotLength 0.30 // meters
#define RobotWidth 0.1  // meters

#define Map_Length 15 // x, meters
#define Map_Width 10  // y, meters
#define Map_X_Resolution 0.05 // meters 
#define Map_Y_Resolution 0.05 // meters



/* DONT CHANGE THESE THINGS */

#define MW Map_Length*PPM // Pool Width
#define MH Map_Width*PPM // Pool Height 

#define XPPC MW / (Map_Length/Map_X_Resolution ) 
#define YPPC MH / (Map_Width/Map_Y_Resolution ) 

#define RH PPM*RobotLength // Boat Height
#define RW PPM*RobotWidth  // Boat Width
#define RCX RW/2.0 // Boat kinematic centre X corodinate 
#define RCY RH-10 // Boat kinematic centre Y corodinate 

#define XRES Map_Length*PPM+2*WT+100
#define YRES Map_Width*PPM+2*WT+100

#define   X1 (XRES-MW)/2
#define   X2 (XRES+MW)/2
#define   Y1 (YRES-MH)/2 
#define   Y2 (YRES+MH)/2




#endif
