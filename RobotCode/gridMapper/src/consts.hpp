#ifndef _CONSTS_HPP_
#define _CONSTS_HPP_

#define NAME "HIAJLAK"

#define LIMITFPS
#define MAXFPS 100  

/* You Can Change These Things */
#define USE_SFML
#define USE_ROS

#define PPM 100 // Pixels per Meter
#define WT 20 // Pool Wall Thickness

#define RobotLength 0.237 // meters
#define RobotWidth 0.162  // meters

// Defines the Reign of Space 
#define Map_X1 -2 // X, meters
#define Map_X2 2 // X, meters
#define Map_Y1 -2  // Y, meters
#define Map_Y2 2 // Y, meters
        
#define Map_XRes 0.05 // meters/cell
#define Map_YRes 0.05// meters/cell

/* DONT CHANGE THESE THINGS */

#define Map_Width ( Map_X2 - Map_X1 )
#define Map_Height ( Map_Y2 - Map_Y1 )

#define MW Map_Width*PPM // Pool Width
#define MH Map_Height*PPM // Pool Height 

#define XPPC MW / (Map_Width/Map_YRes ) 
#define YPPC MH / (Map_Height/Map_XRes ) 

#define RW PPM*RobotLength// Boat Width
#define RH PPM*RobotWidth// Boat Height
#define RCX RW/2.0 // Boat kinematic centre X corodinate 
#define RCY RH/2.0 // Boat kinematic centre Y corodinate 

#define XRES Map_Width*PPM+2*WT+100
#define YRES Map_Height*PPM+2*WT+100

#define   X1 (XRES-MW)/2
#define   X2 (XRES+MW)/2
#define   Y1 (YRES-MH)/2 
#define   Y2 (YRES+MH)/2

#endif
