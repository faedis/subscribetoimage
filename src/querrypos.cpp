
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <unistd.h>
#include <sys/time.h>
extern "C" {
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include "cpiver.h"
#include "ptu.h"
}

using namespace std;

portstream_fd COMstream;			// creat PTU handle



int main( int argc, char** argv ) {
	// Set up PTU +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	int filenumber = 0;
	char COMportName[256] = "/dev/ttyUSB0";
	int BaudRate = 19200;
	// Initialize PTU and set mode

	set_baud_rate(BaudRate);			// set baud rate
	COMstream = open_host_port(COMportName);	// open serial communication
	if (COMstream == PORT_NOT_OPENED)		// check connection is open, else abord
	{
		printf("\nSerial Port setup error.\n");
		return -1;
	}
	else{
		cout << "Serial port to PTU opened\n\n";
	}

	signed short pPos = 0;
	signed short tPos = 0;
	signed short* pPtr = &pPos;
	signed short* tPtr = &tPos;
	char result = 0;
	ros::init(argc,argv, "querrypos_node");
	ros::NodeHandle n;
	ros::Publisher querrypos_pub;
	querrypos_pub = n.advertise<std_msgs::Char>("querrypos",1);
	std_msgs::Char posChar;
	while(true){

		result = get_current_positions(pPtr,tPtr);
		posChar.data = result;
		querrypos_pub.publish(posChar);
	}


	return 0;
}
