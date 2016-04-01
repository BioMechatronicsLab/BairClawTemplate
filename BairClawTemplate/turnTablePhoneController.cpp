/*
 * can_terminal.cpp
 *
 *  Created on: Aug 19, 2010
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <unistd.h>
#include <barrett/os.h>  // For btsleep()
#include <boost/thread.hpp>
#include <barrett/bus/can_socket.h>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

//EPOS REQUIRED
#include "EPOSInterface.h"
//JSON library to manipulate UDP packets
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "rapidjson/filestream.h"   // wrapper of C stream for prettywriter as output

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>

#define BUFLEN 1000
#define PORT 9330 //Port for iphone app


using namespace barrett;
using namespace rapidjson;
using detail::waitForEnter;


int MovementState = 0;

void err(char *str)
{
    perror(str);
    exit(1);
}

//rtcan1 thread communicates with turntable
void readThread(const bus::CANSocket* bus, const bool* going) {
	int ret;
	int id;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;

	while (*going) {
		ret  = bus->receiveRaw(id, data, len, false);
		if (ret == 0) {  // success
			/*printf("(0x%03x)", id);
			for (size_t i = 0; i < len; ++i) {
				printf(" %02x", data[i]);
			}
			printf("\n"); */
			//CHECKS for MovementState
			if(id == 0x581 && data[0] == 0x4b && data[1] == 0x41 && data[2] == 0x60)
			{
				if(data[5] == 0x11){
					MovementState = 2;
				}
				else{
					MovementState = 0; 
				}
			}
		} else if (ret != 1) {  // error other than no data
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}
		usleep(1000);
	}
	return;
}





int main(int argc, char** argv)
{
    
	int port = 1;


	printf("Using CAN bus port %d.\n", port);
	bus::CANSocket bus(port);

	bool going = true;
	boost::thread thread(readThread, &bus, &going);
	

    EPOS2 turntable(1,&bus);
    turntable.reset();
    turntable.enable();
	turntable.SetPositionRegulatorGain( 2000, 80, 800);
	turntable.SetPositionProfile( 4, 2, 2);
    turntable.ActivateProfilePositionMode();
	

    int turntableEncoderCount=0,     magnetState=0     , freeSpin=0;
    int turntableEncoderCountLast=0, magnetStateLast=0 , freeSpinLast=0;
    
    

    //JSON object handling
    char buf[BUFLEN];
    char sendbuf[BUFLEN];
    struct sockaddr_in my_addr, cli_addr;
    int sockfd, sendLen;
    socklen_t slen=sizeof(cli_addr);
    
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        err("socket");
    else
        printf("Server : Socket() successful\n");
    
    bzero(&my_addr, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(PORT);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    
    if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1)
        err("bind");
    else
        printf("Server : bind() successful\n");
    

    
    while(going)
    {
        Document document;
        memset(buf, 0, BUFLEN);
        if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
            err("recvfrom()");
        
        
        if(document.Parse(buf).HasParseError())
        {
            printf("\nParseError\n");
            printf("Original JSON:\n %s\n", buf);
        }
        printf("Original JSON:\n %s\n", buf);
        if(document.IsObject())
        { //JSON success now handle what to do
            //Parse into Variables

            turntableEncoderCount     = document["turntableEncoderCount"].GetInt();
            magnetState               = document["magnetState"].GetInt();
            freeSpin                  = document["freeSpin"].GetInt();
            going                     = document["going"].GetInt();
        }
        
        if(going == 0)
        {
            break;
        }
        else
        {
            if(freeSpin == 1)
            {
                turntable.DisableState();
                btsleep(.1);
                turntable.DigitalOutputOff();
                printf("\nFREESPIN\n");
                
            }
            else if( turntableEncoderCount != turntableEncoderCountLast)
            {
                turntable.enable();
                turntable.ActivateProfilePositionMode();
                turntable.DigitalOutputOff();
                turntable.MoveToPosition(turntableEncoderCount, 0);
                printf("\nMove to postion encoder cound - %d", turntableEncoderCount);
            }
            
            if(magnetState == 1)
            {
                turntable.DigitalOutputOn();
                turntable.DisableState();
                printf("\nMagnet-ON");
            }
            else
            {
                turntable.DigitalOutputOff();
                printf("\nMagnet-OFF");
            }
        }
    
        turntableEncoderCountLast   = turntableEncoderCount;
        magnetStateLast             = magnetState;
        freeSpinLast                = freeSpin;

    }
    //////////////////
    
    
    
    
    /*
    printf("press [Enter] to toggle turntable mag");
    waitForEnter();

    turntable.DigitalOutputOn();
    turntable.DigitalOutputOn();
    turntable.DigitalOutputOn();
    turntable.DigitalOutputOn();
    turntable.DigitalOutputOn();
    turntable.DigitalOutputOn();
    printf("press [Enter] to toggle turntable mag");
    waitForEnter();
    
    turntable.DigitalOutputOff();

    printf("press [Enter] to toggle turntable mag");
    waitForEnter();
    
    turntable.DigitalOutputOn();

    printf("press [Enter] to toggle turntable mag");
    waitForEnter();
    
    turntable.DigitalOutputOff();
    turntable.DisableState();
    */
    /*
    MoveToPosition(EncoderValuesVec[i]);
    MovementState = 1;
    printf("\nMoving");
    while(MovementState != 0)
    {
        GetMovementState(&bus, 0x601);
        btsleep(0.2);
    }
    printf("\nDone Moving: Turning on Magnet");
    DigitalOutputOn(&bus, 0x601);
    DisableState(&bus, 0x601);
    btsleep(0.75);


    EnableEPOS(&bus, 0x601);
    ActivateProfilePositionMode(&bus, 0x601);
    btsleep(2);

		DigitalOutputOff(&bus, 0x601);
	


	btsleep(.5);
	DigitalOutputOff(&bus, 0x601);
	MoveToPosition(&bus, 0x601, 0);
	btsleep(1);
    */
    turntable.DigitalOutputOff();
    turntable.DisableState();
    btsleep(0.2);
	going = false;

	thread.join();

	return 0;
}
