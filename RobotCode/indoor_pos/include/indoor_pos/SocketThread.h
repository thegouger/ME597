/*
 * SocketThread.h
 * Header file for SocketThread.cpp
 */
#ifndef SOCKETTHREAD_H
#define SOCKETTHREAD_H

/*
 * Includes
 */
// General
#include "ros/ros.h"
#include "indoor_pos/ips_msg.h"

//Socket structures
#include <netinet/in.h>

// WAVE specific

//Socket Info
#define PORT_NUMBER 52000
#define SERVER_IP "192.168.1.3" 
#define MAXCOMBUF 80	//size of communication buffer

//Socket Request Types
#define SOCKET_REQ_POS 1

using namespace std;

class SocketThread {
    // Public methods
    public:
        SocketThread(ros::NodeHandle n);
        void Loop();
	//private members
	private:
		//socket communication definitions	
		int sockfd;		//socket file descriptor
		int portno;		//port number
		int numBytes;	//actual number of bytes sent or received
    	struct sockaddr_in serv_addr;	//server address
    	struct hostent *server; 		//host entry pointer containing server information
		char comBuffer[MAXCOMBUF];		//buffer used to communicate over the socket		
        char recBuffer[MAXCOMBUF];		//buffer used to communicate over the socket

		ros::Publisher ips_publisher;
};

#endif 
