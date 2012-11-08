#include <SFML/Graphics.hpp>
#include "consts.hpp"
#include "body.hpp"

LinkedList::LinkedList () {
   head = tail = NULL ;
   size = 0 ; 
}

void
LinkedList::Push (void* Data) {
   ListNode* node = new ListNode ;
   node->data = Data ;
   node->next = head ;
   head = node ;
   size++ ;
}

void*
LinkedList::Pop () {
   if (size < 1) return NULL ;
   ListNode* tmp = head;
   void* Data = tmp->data ;
   head = head->next ;
   size-- ;
   delete tmp ;
   return Data ;
}

void
LinkedList::AddNode (void* Data) {
   Push(Data) ;
}

void*
LinkedList::GetNode (int i) {
   if ( i >= size || i < 0 ) return NULL ;
   ListNode* tmp = head ;
   for (int j=0; j<i; j++) {
      tmp = tmp->next ;
   }
   return tmp->data ;
}

int
LinkedList::GetSize () {
   return size ;
}

LinkedList::~LinkedList () {
   ListNode * tmp ;
   for (int i=0; i<size; i++) {
       tmp = head ;
       head = head->next ;
       delete tmp ;
   }
}

void
LinkedList::RemoveNode (int i) {
   if ( i >= size || i < 0 ) return ;
   if (size ==1) { Pop(); return; }

   ListNode* tmp = head ;
   ListNode* bad ; 
   for (int j=0; j<i-1; j++) {
      tmp = tmp->next ;
   }   
   bad = tmp->next ;
   tmp->next = bad->next ;
   delete bad ;
   size-- ;
}

Transform::Transform () {
   p.x = p.y = a = 0 ;
}

Transform::Transform (float X,float Y,float A) {
   p.x = sqrt(X*X+Y*Y) ; 
   p.y = atan2(Y,X) ;
   a = A ;
}


void 
Transform::Rect (float x1, float y1, float x2, float y2, sf::Color c) {
   surface = sf::Shape::Rectangle (x1,y1,x2,y2,c) ;
}

void
Transform::SetGRotation (float A) {
      surface.SetRotation(A) ; 
   Transform* tmp = (Transform*)children.GetNode(0) ;
   int count = children.GetSize() ;
   sf::Vector2f P = GetGPosition();
   for (int i=0; i<count; i++) {
      tmp = (Transform*)children.GetNode(i) ;
      sf::Vector2f V = tmp->GetLPosition();
      float arad = M_PI*(A)/180 ;
      tmp->SetGPosition(P+sf::Vector2f(V.x*cos(V.y-arad),V.x*sin(V.y-arad))) ;
      tmp->SetGRotation(A+tmp->GetLRotation()) ;
   }
}

void
Transform::SetLRotation (float A) {
    SetGRotation(GetGRotation()+A-a);
    a = A ;
}

void
Transform::SetGPosition ( float x, float y) {
   SetGPosition(sf::Vector2f(x,y));
}

void
Transform::SetGPosition (const sf::Vector2f & P) {
      surface.SetPosition(P) ;
   float count = children.GetSize() ;
   for (int i=0; i<count; i++) {
      Transform* tmp = (Transform*)children.GetNode(i) ;
      sf::Vector2f V = tmp->GetLPosition();
      tmp->SetGPosition(P+sf::Vector2f(V.x*cos(V.y),V.x*sin(V.y))) ;
   }
}

void
Transform::SetLPosition (float X, float Y) {
   float mag2 = sqrt(X*X+Y*Y) ;
   float phi2 = atan2(Y,X) ;
   sf::Vector2f tmpv = sf::Vector2f(mag2*cos(phi2),mag2*sin(phi2)) -
                       sf::Vector2f(p.x*cos(p.y),p.x*sin(p.y)) ;
   SetGPosition(tmpv+GetGPosition()) ;
   p.x = mag2 ;
   p.y = phi2 ;
}

void
Transform::SetLPosition (sf::Vector2f & P) {
   p = P ;
}

void
Transform::SetCenter (float X,float Y) {
   surface.SetCenter(X,Y) ;
}

const sf::Vector2f &
Transform::GetGPosition () {
      return surface.GetPosition() ;
   return GetLPosition() ;
}

float
Transform::GetGRotation () {
      return surface.GetRotation() ; 
   return 0 ;
}

const sf::Vector2f &
Transform::GetLPosition () {
   return p ;
}

float
Transform::GetLRotation () {
   return a ; 
}

bool
Transform::PointOnBody (const sf::Vector2f & G ) {
   return false;
}

void
Transform::AddChild(Transform * T) {
   children.AddNode((void* )T) ;
   SetGPosition(GetGPosition()) ;
   SetGRotation(GetGRotation()) ;
}

int
Transform::RemoveChild(Transform* T) {
   int childcount = children.GetSize();
   for (int i=0; i<childcount; i++) {
      if ( children.GetNode(i) == T ) {
          children.RemoveNode(i) ;
          return 0 ;
      }
   }
   return -1 ;
}

void
Transform::Draw (sf::RenderWindow* W) {
   W->Draw(surface) ;
   int childcount = children.GetSize();
   for (int i=0; i<childcount; i++) {
      Transform* tmp = (Transform*)children.GetNode(i) ;
      tmp->Draw(W) ;
   }
}

Transform::~Transform () {}

