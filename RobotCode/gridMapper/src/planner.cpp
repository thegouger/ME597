#include "consts.hpp"
#include "planner.hpp"

/* --- Path Planning --- */

/* Manhattan Distance Heuristic */
float H(int i, int j, int gi, int gj) {
   return 0.5 * (pow(gi-i,2)+pow(gj-j,2));
}

struct Vector2d {
   float x;
   float y;
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

struct CompareState : public std::binary_function<State*, State*, bool> {
   bool operator()(const State* s1, const State* s2) const {
      return s1->f > s2->f ;
   }
};

bool validPosition(int i, int j) {
   if ( i < 0 || i >= M) return false;
   if ( j < 0 || j >= N) return false;
   if (Map[i][j] > LP0) return false; 
   return true ;
}

State * generateNode(int i, int j,int gi, int gj, float g, State * parent) {
   State * S = new State ;
   S->i = i;   S->j = j;
   g = Map[i][j] > 0.6 ? 100000+g : g;
   S->g = g + Map[i][j];
   S->f = g + H(i,j,gi,gj);
   S->parent = parent;
   return S;
}

vector<Vector2d> * findPath(float goalX,float goalY) {
   int si = (y-Map_BL_y)/Map_Y_Resolution;
   int sj = (x-Map_BL_x)/Map_X_Resolution;
   int gi = (goalY-Map_BL_y)/Map_Y_Resolution;
   int gj = (goalX-Map_BL_x)/Map_X_Resolution;
   
   vector<Vector2d> *path = new vector<Vector2d>;
   State *S; 

   std::priority_queue<State*, vector<State*>, CompareState > pq;
   list<State*> freeList;
    
   pq.push(generateNode(si,sj,gi,gj,0,NULL));

   /* Search for the goal Sate */
   int count = 0 ;
   while ( !pq.empty() ) {
      count++;
      if (count > 1000000) {
         return NULL;
      }
      S = pq.top();
      pq.pop();
      freeList.push_front(S);

      if (S->i == gi && S->j == gj) break; // goal state

      /* Add Neighbours */
      if (validPosition(S->i+1,S->j)) {
         pq.push(generateNode(S->i+1,S->j,gi,gj,S->g,S));
      }
      if (validPosition(S->i-1,S->j)) {
         pq.push(generateNode(S->i-1,S->j,gi,gj,S->g,S));
      }
      if (validPosition(S->i,S->j+1)) {
         pq.push(generateNode(S->i,S->j+1,gi,gj,S->g,S));
      }
      if (validPosition(S->i,S->j-1)) {
         pq.push(generateNode(S->i,S->j-1,gi,gj,S->g,S));
      }
   }

   /* Form the Path */
   if (S != NULL && S->i == gi && S->j == gj) {
      while (S != NULL) {
         Vector2d p; 
         p.x = Map_BL_x + Map_X_Resolution*S->j; 
         p.y = Map_BL_y + Map_Y_Resolution*S->i;
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
