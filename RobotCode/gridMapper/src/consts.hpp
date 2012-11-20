#ifndef _CONSTS_HPP_
#define _CONSTS_HPP_

#define NAME "HIAJLAK"

#define LIMITFPS
#define MAXFPS 100  

/* You Can Change These Things */
#define USE_SFML
#define USE_ROS
#ifdef USE_ROS
   #define USE_SIMULATOR
#endif

#define PATH_PLANNER 0 // 0: wavefront, 1: A*, 2: Both

#define PPM 100 // Pixels per Meter
#define WT 20 // Pool Wall Thickness
#define PathThickness 2 // Pixels

#define RobotLength 0.237 // meters
#define RobotWidth 0.162  // meters

// Defines the Reign of Space 
#define Map_X1 -1 // X, meters
#define Map_X2 6 // X, meters
#define Map_Y1 -2  // Y, meters
#define Map_Y2 2 // Y, meters
        
#define Map_XRes 0.05 // meters/cell
#define Map_YRes 0.05// meters/cell

#define wall_tol 2.0*RobotWidth/Map_XRes
#define goal_tol RobotWidth/1.5

/* DONT CHANGE THESE THINGS */

#define Map_Width ( Map_X2 - Map_X1 )
#define Map_Height ( Map_Y2 - Map_Y1 )

#define MW Map_Width*PPM // Pool Width
#define MH Map_Height*PPM // Pool Height 

#define XPPC MW / (Map_Width/Map_XRes ) 
#define YPPC MH / (Map_Height/Map_YRes ) 

#define RW PPM*RobotWidth// Boat Width
#define RL PPM*RobotLength// Boat Height
#define RCX RL/2.0 // Boat kinematic centre X corodinate 
#define RCY RW/2.0 // Boat kinematic centre Y corodinate 

#define XRES Map_Width*PPM+2*WT+100
#define YRES Map_Height*PPM+2*WT+100

#define   X1 (XRES-MW)/2
#define   X2 (XRES+MW)/2
#define   Y1 (YRES-MH)/2 
#define   Y2 (YRES+MH)/2

#endif
