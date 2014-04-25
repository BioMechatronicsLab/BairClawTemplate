/*=========================================================================
| (c) 2013  Arizona State University Biomechatronics Lab
|--------------------------------------------------------------------------
| Project : Bair Claw Main/Interface
| File    : BairClawMain.c
| Authors : Randy Hellman (RBHellman@gmail.com, RHellman@asu.edu)
			Veronica Santos (Veronica.Santos@asu.edu)
|--------------------------------------------------------------------------
| Function: Main interface for BairClaw controls CAN communication between 
| 			BairClaw and EPOS motor controllers. 
|--------------------------------------------------------------------------
 ========================================================================*/

#include <iostream>
#include <fstream>
#include <sstream>
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


#include <iostream>
#include <string>
#include <cstdio>

#include <unistd.h>

#include <boost/thread.hpp>
#include <barrett/bus/can_socket.h>

#include "EPOSInterface.h"


using namespace barrett;

const int FEnode = 2; //Constant to specifies node 1 to index Flextion extension
const int PIPDIPnode = 1; 
bus::CANSocket bus1(1);
int A1set = 0, A2set = 0;
EPOS2 FE(FEnode, &bus1), PIPDIP(PIPDIPnode, &bus1), EPOS2all(3, &bus1);
EPOS2* currentEPOSController; 
BairClawDigit indexFinger(3, 2, 1, &bus1);

void readThread(const bus::CANSocket* bus, bool* going) {
	int ret;
	int id, SDO;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;

	while (*going) {
		ret  = bus->receiveRaw(id, data, len, false);
		if (ret == 0) {  // success
			SDO = data[1] + (data[2] << 8); // SERVICE DATA OBJECT (SDO)
			if(id == 0x201) //CANid of index finger set on arduino/CANbusShield.  
			{
				indexFinger.set(data); // Assignes joint data to object given data[]. 
				/*if(!indexFinger.limitsOk()){
					EPOS2all.resetAll();
				}*/	
			}
			else
			{ 
				if(id == FE.nodeRec )
				{
					currentEPOSController = &FE;
				}
				else if(id == PIPDIP.nodeRec)
				{
					currentEPOSController = &PIPDIP;
				}
				else if(id == EPOS2all.nodeRec)
				{
					currentEPOSController = &EPOS2all;
				}

				if(data[2] == 0x20)
				{
					if ( SDO == 0x207c ){
						if      (data[3] == 0x01)
						{
							currentEPOSController->A1 = data[4] + (data[5] << 8);
						}
						else if (data[3] == 0x02)
						{
							currentEPOSController->A2 = data[4] + (data[5] << 8);
						}
					}
				}	
				if ( SDO == 0x6064)
					currentEPOSController->currPos = data[4] + (data[5] << 8) + (data[6] << 16) + (data[7] << 24);
				
			}
		} else if (ret != 1) {  // error other than no data
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}

		usleep(500);
	}
}


int main(int argc, char** argv) {
	int port  = 0;
	int count = 0;
	switch (argc) {
	case 1:
		printf("No port argument given. Using default.\n");
		break;
	case 2:
		port = atoi(argv[1]);
		break;
	default:
		printf("ERROR: Expected 1 or 0 arguments.\n");
		return -1;
		break;
	}

	printf("Using CAN bus port %d.\n", port);
	bus::CANSocket bus1(port);

	bool going = true;
	boost::thread thread(readThread, &bus1, &going);



//////////////////////////////////////////////////////////////////////////////////////////
	
	usleep(500000);
	FE.enable();
	indexFinger.motorFE.SetCurrentLimit(200);
//	PIPDIP.enable();
	EPOS2all.enable();
	EPOS2all.SetCurrentLimit(200);
/*  Uncomment to test Backdrivability mode! 
	indexFinger.motorFE.ActivateCurrentMode();
	indexFinger.setStaticFriction();
	indexFinger.backDrive();
*/
	printf("Press [ENTER] to reset nodes and end program\n");
	detail::waitForEnter();
//	indexFinger.init();
//	indexFinger.vis();

	EPOS2all.SetPositionProfile(100,10000,10000);
	EPOS2all.ActivateProfilePositionMode();

	indexFinger.motorFE.SetCurrentLimit(500);
	indexFinger.motorFE.enable();
	indexFinger.motorFE.SetPositionProfile(500,1000,1000);
	indexFinger.motorFE.ActivateProfilePositionMode();

	indexFinger.motorPIPDIP.SetCurrentLimit(500);
	indexFinger.motorPIPDIP.enable();
	indexFinger.motorPIPDIP.SetPositionProfile(300,1000,1000);
	indexFinger.motorPIPDIP.ActivateProfilePositionMode();

	double desiredPos = 5, desiredPosPIPDIP = 10;

	double changeInMotorPosFE = 0, changeInMotorPosPD = 0;
	int sw=0;
	while (going)
	{	
		changeInMotorPosFE = (indexFinger.FEPercent - desiredPos) * 50;
		changeInMotorPosPD = ((indexFinger.PIPPercent + indexFinger.DIPPercent) - desiredPosPIPDIP) * 50;
		if(count % 2 == 0)
			indexFinger.motorFE.MoveToPosition(changeInMotorPosFE, 1);
		else
			indexFinger.motorPIPDIP.MoveToPosition(changeInMotorPosPD, 1);

		count++;
		FE.readAnalog();
		PIPDIP.readAnalog();
		indexFinger.calcPercentage();
		
		if(count % 5 == 0){ //if set to 1 does nothing just left in for quick changes
			system("clear");
			indexFinger.print();
			printf("\nchangeInMotorPosFE = %6.2f, desiredPos = %4.2f, indexFinger.FEPercent = %4.2f count = %d\n",changeInMotorPosFE, desiredPos, indexFinger.FEPercent, count);
			printf("\nchangeInMotorPosPD = %6.2f, desiredPos = %4.2f, PIPPercent+DIPPercent = %4.2f count = %d\n",changeInMotorPosPD, desiredPos, indexFinger.PIPPercent+indexFinger.DIPPercent, count);
		}
		if(count % 100 == 0){
			sw++;
			if(sw % 2 == 0 ){
				desiredPos = 15;
				desiredPosPIPDIP = 70;	
			}else{
				desiredPos = 5;
				desiredPosPIPDIP = 10;
			}

		}
		usleep(5000);
	}
//////////////////////////////////////////////////////////////////////////////////////////	

/*	FE.reset();
	usleep(1000000);
	PIPDIP.reset();
	usleep(1000000);
	FE.enable();
	PIPDIP.enable();
	usleep(1000000);
	EPOS2all.resetAll();
*/
	EPOS2all.resetAll();

	going = false;
	thread.join();

	return 0;
}
