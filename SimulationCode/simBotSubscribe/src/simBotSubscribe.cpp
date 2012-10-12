#include <iostream>
#include <stdlib>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

class SimBotSubscribe
{
   private:
      Twist _robot_twist;
      NodeHandle _node_handle;
      Subscriber _subscriber;
   
   public:
      SimBotSubscribe()
      {
         _subscriber = _node_handle.subscribe(/*Twist,cmd_vel, 1...*/);
      }
}

int main(int argv, char *argv[])
{
   init() // ...
