#ifndef _CONSTS_HPP_
#define _CONSTS_HPP_

#define NAME "HIAJLAK"
//*
#define LIMITFPS
#define MAXFPS 100  
//*/

/* You Can Change These Things */

#define PPM 100 // Pixels per Meter
#define WT 20 // Pool Wall Thickness

#define RobotLength 0.30 // meters
#define RobotWidth 0.2  // meters

// Defines the Reign of Space 
#define Map_BL_x -2.5 // X, meters
#define Map_BL_y -2.5 // Y, meters
#define Map_TR_x 2.5 // X, meters
#define Map_TR_y 2.5 // Y, meters
        
#define Map_X_Resolution 0.2 // meters/cell
#define Map_Y_Resolution 0.2// meters/cell

/* DONT CHANGE THESE THINGS */

#define Map_Length ( Map_TR_x - Map_BL_x )
#define Map_Width ( Map_TR_y - Map_BL_y )

#define MW Map_Width*PPM // Pool Width
#define MH Map_Length*PPM // Pool Height 

#define XPPC MW / (Map_Width/Map_Y_Resolution ) 
#define YPPC MH / (Map_Length/Map_X_Resolution ) 

#define RH PPM*RobotLength // Boat Height
#define RW PPM*RobotWidth  // Boat Width
#define RCX RW/2.0 // Boat kinematic centre X corodinate 
#define RCY RH-10 // Boat kinematic centre Y corodinate 

#define XRES Map_Width*PPM+2*WT+100
#define YRES Map_Length*PPM+2*WT+100

#define   X1 (XRES-MW)/2
#define   X2 (XRES+MW)/2
#define   Y1 (YRES-MH)/2 
#define   Y2 (YRES+MH)/2




#endif
