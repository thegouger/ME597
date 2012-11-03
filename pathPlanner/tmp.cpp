#include <list>
#include <queue>
#include <vector>
#include <functional>

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