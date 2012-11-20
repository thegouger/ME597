#ifndef _BODY_HPP_
#define _BODY_HPP_

#include <SFML/Graphics.hpp>


struct ListNode {
   void * data ;
   ListNode * next ;
} ;

class LinkedList {
   ListNode * head ;
   ListNode * tail ;
   int size ; 
public:
   LinkedList () ;
   
   void Push (void * ) ;
   void* Pop () ;
   void* GetNode (int i) ;
   void RemoveNode (int ) ;
   void AddNode (void*) ;
   int GetSize () ;
   
   ~LinkedList();

} ;


class Transform {

   private:
   sf::Vector2f p ;
   float a ;
   LinkedList children ;
   sf::Shape surface ;

   public:
   Transform () ;
   Transform (float X,float Y,float A) ;

   void Rect (float x1, float y1, float x2, float y2, sf::Color c) ;
   void Rect (float x1, float y1, float x2, float y2, sf::Color c,float outline,sf::Color oc) ;

   void SetGPosition (float ,float) ;
   void SetGPosition (const sf::Vector2f & ) ;
   void SetGRotation (float ) ;
   void SetLPosition (float ,float) ;
   void SetLPosition (sf::Vector2f & ) ;
   void SetLRotation (float ) ;
   void SetCenter (float X,float Y) ;
    
   const sf::Vector2f & GetGPosition () ;
   float GetGRotation () ;
   const sf::Vector2f & GetLPosition () ;
   float GetLRotation () ;

   bool PointOnBody(const sf::Vector2f & ) ;

   void AddChild (Transform* ) ;
   int RemoveChild (Transform* ) ;

   void Draw (sf::RenderWindow* );

   ~Transform();

} ;


#endif
