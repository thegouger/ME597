#include <SFML/Graphics.hpp>
#include "consts.hpp"
#include "body.hpp"
#include <iostream>
#include <math.h>
#include <list>
#include <queue>
#include <vector>
#include <functional>

using namespace std;

/* All The Colours */
sf::Color RobotColor(32,32,32);
sf::Color PathColor(255,30,32);

sf::Color Border(32,32,32);
sf::Color BG(106,106,67);
sf::Color Empty(86,105,106);
sf::Color Full(30,30,32);
sf::Color Unknown(126,35,35);
/* ^^ Colours ^^ */

float x,y,theta;

const int M = Map_Length/Map_X_Resolution;
const int N = Map_Width/Map_Y_Resolution; 
float Map[M][N];

/* straight line Heuristic */
float H(int i, int j, int gi, int gj) {
   return (pow(gi-i,2)+pow(gj-j,2));
   return 0 ; 
}

struct Vector2i {
   float i;
   float j;
};

class State {
   public:
   State() {};
   bool operator>(const State& right) const;
   bool operator<(const State& right) const;
   int i;
   int j;
   float g;
   float f;
   State * parent;
};

bool State::operator<(const State& right) const {
   return f > right.f;
}
bool State::operator>(const State& right) const {
   return f > right.f;
}

struct CompareState : public std::binary_function<State*, State*, bool> {
   bool operator()(const State* s1, const State* s2) const {
      return s1->f > s2->f ;
   }
};

State * generateNode(int i, int j,int gi, int gj, float g, State * parent) {
   State * S = new State ;
   S->i = i;
   S->j = j;
   S->g = g;
   S->f = g + H(i,j,gi,gj);
   S->parent = parent;
   return S;
}


bool validPosition(int i, int j) {
   if ( i < 0 || i >= M) return false;
   if ( j < 0 || j >= N) return false;
   return true ;
}

vector<Vector2i> * findPath(float goalX,float goalY) {
   int si = (y-0)/Map_Y_Resolution;
   int sj = (x-0)/Map_X_Resolution;
   int gi = (goalY-0)/Map_Y_Resolution;
   int gj = (goalX-0)/Map_X_Resolution;
   
   vector<Vector2i> *path = new vector<Vector2i>;
   State *S; 

   std::priority_queue<State*, vector<State*>, CompareState > pq;
   list<State*> freeList;
    
   pq.push(generateNode(si,sj,gi,gj,0,NULL));

   /* Search for the goal Sate */
   while ( !pq.empty() ) {
      S = pq.top();
      pq.pop();
      freeList.push_front(S);

      if (S->i == gi && S->j == gj) break; // goal state

      /* Add Neibours */
      if (validPosition(S->i+1,S->j)) {
         pq.push(generateNode(S->i+1,S->j,gi,gj,S->g+1,S));
      }
      if (validPosition(S->i-1,S->j)) {
         pq.push(generateNode(S->i-1,S->j,gi,gj,S->g+1,S));
      }
      if (validPosition(S->i,S->j+1)) {
         pq.push(generateNode(S->i,S->j+1,gi,gj,S->g+1,S));
      }
      if (validPosition(S->i,S->j-1)) {
         pq.push(generateNode(S->i,S->j-1,gi,gj,S->g+1,S));
      }
   }

   /* Form the Path */
   if (S != NULL && S->i == gi && S->j == gj) {
      while (S != NULL) {
         Vector2i p; p.i=S->i; p.j=S->j;
         path->insert(path->begin(),p);
         S = S->parent;
      }
   }

   /* Free all the memory */
   while ( !freeList.empty() ) {
      S = freeList.front();
      freeList.pop_front();
      delete S;
   }

   /* Return the Path */
   return path;
}




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

void fillMap() {
   for (int i=0; i<M; i++) {
      for (int j=0; j<N; j++) {
        if (j % 15 > 10 ) {
            Map[i][j] = 0.1f;
         } else if (j % 15 > 5) {
            Map[i][j] = 0.9f;
         }
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

void drawPath(vector<Vector2i> * path, sf::RenderWindow *W) {
   sf::Shape Cell;
   for (int i=0; i<path->size(); i++) {
      Cell = sf::Shape::Rectangle (X1+path->at(i).i*XPPC, Y1+path->at(i).j*YPPC, X1+(path->at(i).i+1)*XPPC, Y1+YPPC*(path->at(i).j+1),PathColor); 
      W->Draw(Cell);
   }
}


int main () {
   sf::RenderWindow Window(sf::VideoMode(XRES,YRES,32),NAME) ;
   #ifdef LIMITFPS
      Window.SetFramerateLimit(MAXFPS) ;
   #endif

   /* ----- Setup ----- */
   initialiseMap();
//   fillMap();   
   Transform Robot;
   Robot.Rect(0,0,RW,RH,RobotColor);
   Robot.SetCenter(RCX,RCY);

   x = 3; // Get These From ROS
   y = 3;
      Window.Clear(BG) ;
      /* ------ Loop ------ */
      drawMap(&Window);
   
/*   for (int i=0; i<path->size(); i++) {
      cout << path->at(i).i << ", " << path->at(i).j << "\n";
   } */

   theta = 0;
   float rad = 3.14159 /180 ; 
   int k = 180;
   float amp = 2;
   /* ------------------------ */
   while( Window.IsOpened() ) {
      Window.Clear(BG) ;
      /* ------ Loop ------ */
      //drawMap(&Window);
      
      y = 7.5 + amp * sin(k*rad);
      x = 5  + amp * cos(k*rad);
      //x = 6; y = 6; 
      k++;

      Robot.SetGPosition(X1+PPM*y,Y1+PPM*x);
      Robot.SetGRotation(theta);
      Robot.Draw(&Window);
      //*
      vector<Vector2i> * path = findPath(5,7.5);
      drawPath(path,&Window);
      delete path;
      //*/
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
