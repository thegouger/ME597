#ifndef __PLANNER_HPP__
#define __PLANNER_HPP__

#include <vector>

std::vector<Vector2d> * findPath(float goalX,float goalY) {
 
struct Vector2d {
   float x;
   float y;
};

class State {
   public:
   State() {};
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

#endif
