/*
 * SocketThread.cpp
 * Thread responsible for communication with laptop/PC
 * This is the TCP Socket Client
 */

// General includes
#include <typeinfo>
#include <iostream>
#include <pthread.h>
#include <string>       // C++ 'string'
#include <sys/time.h>   // Time retrieval functions
#include <time.h>       // Time retrieval functions
#include <stdlib.h>

//Socket includes
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <string.h>

#include "indoor_pos/SocketThread.h"

using namespace std;


/**
 * Subclass of WaveThread constructor
 * @param period Period (in ms) of thread
 * @param name Descriptive of thread
 * @param start_time Program start time
 * @param pub Instance of message publishing class
 */
SocketThread::SocketThread(ros::NodeHandle n)
{
    //create and open socket to server
    portno = PORT_NUMBER;
	ROS_INFO("Indoor_Pos:Opening socket");
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0){
        ROS_INFO("Indoor_Pos:ERROR connecting");
//        cerr << "ERROR opening socket" << endl;
    }
    server = gethostbyname(SERVER_IP);
    if (server == NULL) {
        ROS_INFO("Indoor_Pos:ERROR connecting");
  //      fprintf(stderr,"ERROR, no such host\n");
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        ROS_INFO("Indoor_Pos:ERROR connecting");

	ips_publisher=n.advertise<indoor_pos::ips_msg>("indoor_pos", 1000);

}

/**
 * Override of base Loop() function
 * In this case, an infinite loop which publishes
 * and receives message. Probably want to do something like this
 */
void SocketThread::Loop() 
{	
    char * pEnd;
    double posX, posY, yaw; //, velX, velY;
    indoor_pos::ips_msg current_pos;

    // Interface loop

    //Receive position data from Laptop
        //Receive position data from Laptop
	int bytesRead = 0;
    bzero(comBuffer,MAXCOMBUF);	
	do
	{	
		bzero(comBuffer,MAXCOMBUF);
		numBytes = read(sockfd,comBuffer,1);
		
		if (numBytes < 0) 
		{
		    cerr << "ERROR reading from socket" << endl;
		    break;
		}
	} while (ros::ok() && comBuffer[0] != 's');

	while (ros::ok() && bytesRead < MAXCOMBUF)
	{
		//Receive position data from Laptop
		bzero(comBuffer,MAXCOMBUF);		
		numBytes = read(sockfd,comBuffer,MAXCOMBUF-bytesRead);
		if (numBytes < 0) 
		{
		    cerr << "ERROR reading from socket" << endl;
		    break;
		}
		memcpy(recBuffer+bytesRead,comBuffer,numBytes);
		bytesRead = bytesRead + numBytes;
	} 
    //Parse received message for position data
    posX = strtod(recBuffer, &pEnd);
    posY = strtod(pEnd, &pEnd);
    yaw = strtod(pEnd, &pEnd);

    current_pos.X = posX;
    current_pos.Y = posY;
    current_pos.Yaw = yaw;
    ips_publisher.publish(current_pos);
    //ROS_INFO("Indoor_Pos: X:%f, Y:%f, Yaw:%f",posX,posY,yaw);
}


int main(int argc, char **argv)
{

	ros::init(argc,argv,"main_control");

	ros::NodeHandle n;

	SocketThread main_socket(n);


	ros::Rate loop_rate(10);		//20Hz update rate
	while (ros::ok())
  	{	
		main_socket.Loop();
		//ROS_INFO("Looping!");
	}

}

