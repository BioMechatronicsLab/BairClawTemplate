/*
 * two_wams.cpp
 *
 *  Created on: Feb 15, 2011
 *      Author: dc
 */

#include <iostream>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/os.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
//JSON library to manipulate UDP packets
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "rapidjson/filestream.h"   // wrapper of C stream for prettywriter as output
#include <cstdio>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#define BUFLEN 1000
#define PORT 9330


using namespace barrett;
using namespace rapidjson;
using detail::waitForEnter;
char  c ='c';
bool  shouldKeepRunning = true;
bool  shouldInitHand = false;
bool  shouldGravityComp = false;
bool  shouldHoldPosition = false;
bool  shouldSendWAMHome = false;
bool  shouldIterate = false;

bool WAMg = false;
bool WAMh = false;
bool WAMs = false;
typedef Hand::jp_type hjp_t;

boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);
void handIPadController(ProductManager& pm0, ProductManager& pm1);

void err(char *str)
{
    perror(str);
    exit(1);
}


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm0;
	//ProductManager pm1("/etc/barrett/bus1/default.conf");

	printf("Starting the WAM on Bus 0...\n");
	boost::thread wt0 = startWam(pm0, wamThread0<4>, wamThread0<7>);
	printf("Starting the WAM on Bus 1...\n");
	//boost::thread wt1 = startWam(pm1, wamThread1<4>, wamThread1<7>);
    boost::thread handIPad(handIPadController, boost::ref(pm0), boost::ref(pm0));
    
    printf("Press [ENTER] InitHandFrom hand thread");
    waitForEnter();
    WAMh = true;
    

    wt0.join();
    handIPad.join();
	//wt1.join();

	return 0;
}


void handIPadController(ProductManager& pm0, ProductManager& pm1)
{
    //HandVariables
    int hand0_shouldInit(0), hand1_shouldInit(0);
        //WAM hand setup
    if ( !pm0.foundHand() ) {
        printf("ERROR: No Hand found on bus!\n");
    }
    std::cout << "FOUND Hand on bus - " << pm0.getWamDefaultConfigPath() << "  -" << std::endl;
    Hand& hand0 = *pm0.getHand();

    

    
    //JSON object handling
    char buf[BUFLEN];
    struct sockaddr_in my_addr, cli_addr;
    int sockfd;
    socklen_t slen=sizeof(cli_addr);
    //char buf[BUFLEN];
    
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
    
    
    
    while (pm0.getSafetyModule()->getMode() == SafetyModule::ACTIVE)
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

        if(document.IsObject())
        { //JSON success now handle what to do
            printf("hand0_spread = %g\n", document["hand0_spread"].GetDouble());
            printf("hand1_spread = %g\n", document["hand1_spread"].GetDouble());
            hand0_shouldInit = document["hand0_shouldInit"].GetInt();
            printf("hand0_shouldInit = %d\n", hand0_shouldInit);
            const Value& hand0_finger = document["hand0_finger"]; // Using a reference for consecutive access is handy and faster.
            assert(hand0_finger.IsArray());
            for (SizeType i = 0; i < hand0_finger.Size(); i++) // rapidjson uses SizeType instead of size_t.
                printf("hand0_finger[%d] = %4.4f\n", i, hand0_finger[i].GetDouble());
            
            
            const Value& hand1_finger = document["hand1_finger"]; // Using a reference for consecutive access is handy and faster.
            assert(hand1_finger.IsArray());
            for (SizeType i = 0; i < hand1_finger.Size(); i++) // rapidjson uses SizeType instead of size_t.
                printf("hand1_finger[%d] = %4.4f\n", i, hand1_finger[i].GetDouble());
            
            
            
            
            if(hand0_shouldInit)
            {
                hand0.initialize();
            }
            else
            {
                double fingerPos    = hand0_finger[SizeType(0)].GetDouble();
                double fingerSpread = hand0_finger[SizeType(3)].GetDouble();
                hand0.velocityMove(hjp_t(fingerPos,fingerPos,fingerPos,fingerSpread), Hand::WHOLE_HAND);
            }
            
        }
        else
        {
            printf("!IsObject()");
        }
        
        printf("\nUDP recieve\n");
        
    }
    
    close(sockfd);
    

}

boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7)
{
	pm.waitForWam();
	pm.wakeAllPucks();

	if (pm.foundWam4()) {
		return boost::thread(wt4, boost::ref(pm), boost::ref(*pm.getWam4()));
	} else {
		return boost::thread(wt7, boost::ref(pm), boost::ref(*pm.getWam7()));
	}
}


template <size_t DOF> void wamThread0(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	/*
    wam.gravityCompensate();
    Hand& hand = *pm.getHand();
    typedef Hand::jp_type hjp_t;
    double OR = -0.75;
	double CR = 0.75;
    Hand::jv_type opening(OR);
	Hand::jv_type closing(CR);
	double O = 0.0;
	double C = 2.4;
	double SC = M_PI;
	hjp_t open(O);
	hjp_t closed(C);
	closed[3] = SC;
    
	jp_type jp(0.0);
	wam.moveTo(jp);
    btsleep(1);
    
    
    
    while(!shouldInitHand)
    {
        btsleep(0.1);
    }
    if ( !pm.foundHand() ) {
        printf("ERROR: No Hand found on bus!\n");
    }
    else
    {
        std::cout << "FOUND Hand on bus - " << pm.getWamDefaultConfigPath() << "  -" << std::endl;
        hand.initialize();
        hand.close();
        hand.trapezoidalMove(open, Hand::GRASP);

    }
    double spreadOpen = 0.0;

    while(shouldKeepRunning)
    {
        while(!shouldIterate)
        {
            btsleep(0.01);
        }
        if(WAMg)
        {
            wam.idle();
            std::cout << "WAM THREAD GRAVITYCOMP" << std::endl;
        }
        else if(WAMh)
        {
            wam.moveTo(wam.getJointPositions());
        }
        else if(WAMs)
        {
            spreadOpen += 0.1;
            hjp_t spreadState(spreadOpen);
            hand.trapezoidalMove(spreadState, Hand::SPREAD);
        }
        btsleep(1);
        WAMg = false;
        WAMh = false;
        WAMs = false;
        shouldIterate=false;
        
        
    }
    
    hand.close();
    hand.trapezoidalMove(open, Hand::GRASP);
    hand.waitUntilDoneMoving();
    
    while(!shouldSendWAMHome)
    {
        btsleep(0.1);
    }
    wam.moveHome();
     */
    std::cout << std::endl << "Shift-idle to end program" << std::endl;
    while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {

		sleep(1);
	}

}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}




















