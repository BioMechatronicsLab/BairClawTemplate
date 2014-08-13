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
#ifndef _EPOSInterface_h
#define _EPOSInterface_h

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <string>
#include <cstdio>

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


#define MAXCURRENTALLOWED 1200 /**< Detailed description after the member */
#define MAX_FOLLOWING_ERROR 99999999 //Used to prevent faluts in positon control moded

namespace barrett{



class EPOS2{
	
	const bus::CANSocket* bus; // Points to can socket that you are using and can only
							   // be assigned when initializing. 
protected: 
	int node;
public:
    int currDemand;
    int currSetCount, currSetError;
	int A1, A2; //Analog read values 0-5000mv
	int currPos;
	int A1set, A2set; //Logs how many times the analog voltage has been set for each.
	bool A1flag, A2flag; //Logs that each are being updated when requested.
    bool isEnabled;
    int staticFrictionF, staticFrictionE;
	int nodeRec;
	int nodeSet;
	// Constructor. Minimal error checking be carefule and make sure you 
	// know what you are doing!!
	EPOS2(int nodeSetassign, const bus::CANSocket* busSet){
        A1 = 0;
        A2 = 0;
		nodeSet = nodeSetassign;
		node = 0x600 + nodeSet;
		nodeRec = 0x580 + nodeSet;
		bus = busSet;
		currDemand = 0;
		currPos = 0;
        currSetCount = 0;
        currSetError = 0;
        staticFrictionF = 60;
        staticFrictionE = 60;
        isEnabled = false;
	};

	void enable();
    int  getNode();
	void readAnalog();
	void readAnalog1();
	void readAnalog2();
	void setA1(); // Need to add to manage flag update 
	void setA2(); // "    "
	void reset();
	void reset(int node);
	void resetAll();
	void MoveToPosition(int pos, int controlWorld);
    void SetMaxFollowingError(int MaxFollowingError);
	void GetMovementState();
	void GetPosition();
	void SetPositionProfile(int Velocity, int MaxAccel, int MaxDecel);
	void SetPositionRegulatorGain(int P, int I, int D);
	void DigitalOutputOn();
	void DigitalOutputOff();
	void DisableState();
	void ActivateProfilePositionMode();
	void ActivateCurrentMode(int continuousCurrentLimit, int outputCurrentLimit, int MaxSpeed);
    /** \returns the inner size.
     * \note member function that sets motor current to desired (int)mA value.
     * will not exceed MAXCURRENTALLOWED that is defined in EPOSInterface.h
     */
	void SetCurrent(int setCurrent);
	void SetCurrentLimit(int setCurrentLimit);
	int  CurrDemanValue(){return currDemand;}

	void print(){
		printf("A1 = %d,  A2 = %d, A1set = %d A2set = %d\n",\
		 A1, A2, A1set, A2set);
	}
};




}; //From namespace barrett{}


#endif
