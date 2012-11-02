#include <SFML/Graphics.hpp>
#include "consts.hpp"
#include "body.hpp"
#include <iostream>

using namespace std;

/* All The Colours */
sf::Color RobotColor(32,32,32);

sf::Color Border(32,32,32);
sf::Color BG(106,106,67);
sf::Color Empty(86,105,106);
sf::Color Full(30,30,32);
sf::Color Unknown(126,35,35);
/* ^^ Colours ^^ */


const int M = Map_Length/Map_X_Resolution;
const int N = Map_Width/Map_Y_Resolution; 
float Map[M][N];

float unlogit(float p) {
   return exp(p)/(1+exp(p));
}

void initialiseMap() {
   for (int i=0; i<M; i++) {
      for (int j=0; j<N; j++) {
         Map[i][j] = 0.1f;
      }
   }
}

sf::Color & colorTransform (float p) {
   sf::Color C;
   if (p > 0.6) {
      return Full;
   }
   else if (p < 0.4) {
      return Empty;
   }
   return Unknown;
}

/* Here is where we draw the map */
void drawMap(sf::RenderWindow * W) {
   sf::Shape Cell;
   sf::Color C; 
   W->Draw(sf::Shape::Rectangle(X1-WT,Y1-WT,X2+WT,Y2+WT,Border));
   for (int i=0; i<M; i++) {
      for (int j=0; j<N; j++) {
         C = colorTransform(Map[i][j]);
         Cell = sf::Shape::Rectangle (X1+i*XPPC, Y1+j*YPPC, X1+(i+1)*XPPC, Y1+YPPC*(j+1),C); 
         W->Draw(Cell);
      }
   }
   
}


int main () {
   sf::RenderWindow Window(sf::VideoMode(XRES,YRES,32),NAME) ;
   #ifdef LIMITFPS
      Window.SetFramerateLimit(MAXFPS) ;
   #endif

   /* ----- Setup ----- */
   initialiseMap();
   
   Transform Robot;
   Robot.Rect(0,0,RW,RH,RobotColor);
   Robot.SetCenter(RCX,RCY);

   float x = 5; // Get These From ROS
   float y = 5;
   float theta = 45;
   /* ------------------------ */
   while( Window.IsOpened() ) {
      Window.Clear(BG) ;
      /* ------ Loop ------ */
      drawMap(&Window);
      
      Robot.SetGPosition(X1+PPM*x,Y1+PPM*y);
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
   }
   return EXIT_SUCCESS ; 
}
