//
//  BairClawInterface.h
//  
//
//  Created by Randy Hellman on 10/31/13.
//
//

#ifndef _BairClawInterface_h
#define _BairClawInterface_h
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



namespace barrett{


class BCDigit {
	int setCounter;
	int JointShutOffRange;
public:
	EPOS2 motor;
	bool isInit;
	int AdAb, FE, PIP, DIP;
	int AdAbmin, FEmin, PIPmin, DIPmin;
	int AdAbmax, FEmax, PIPmax, DIPmax;
	double AdAbPercent, FEPercent, PIPPercent, DIPPercent;
	double AdAbRange, FERange, PIPRange, DIPRange;
	int normalRotation;                   //used to determine is flextion is pos mA. Set to neg(-) if flipped
	int staticFrictionF, staticFrictionE; //used to set static friction for each joint during initilization.
    std::string name;                     //name assined to BCdigit as a descrpiter
    
	// Constructor. Minimal error checking be carefule and make sure you
	// know what you are doing!!
	BCDigit(int node, const bus::CANSocket* busSet): motor( node, busSet)
	{
		AdAb = 0; AdAbmin = 18 ; AdAbmax = 1020;
		FE   = 0; FEmin   = 392; FEmax   = 1021;
		PIP  = 0; PIPmin  = 2  ; PIPmax  = 790;
		DIP  = 0; DIPmin  = 611; DIPmax  = 832;
		setCounter = 0;
		isInit = 0;
		JointShutOffRange = 5;
		normalRotation = 0; //Default to normal rotation (+) mA for flextion
	}
    
    
	void set(unsigned char data[]){
		AdAb = data[0] + (data[1] << 8);
		FE   = data[2] + (data[3] << 8);
		PIP  = data[4] + (data[5] << 8);
		DIP  = data[6] + (data[7] << 8);
	}
    
	void init();
	void vis ();
	void calcPercentage();
	void setStaticFriction();
	void backDrive();
	/*
     int  limitsOk(){
     
     int limit=1;
     if( (AdAb > AdAbmax-JointShutOffRange) || (AdAb < AdAbmin+JointShutOffRange) ){
     limit =0;
     }else if( (FE > FEmax-JointShutOffRange) || (FE < FEmin+JointShutOffRange) ){
     limit =0;
     }else if( (PIP < PIPmin+JointShutOffRange) && (DIP > DIPmax-JointShutOffRange) ){
     limit =0;
     }else if( (PIP > PIPmax-JointShutOffRange) && (DIP < DIPmin+JointShutOffRange) ){
     limit =0;
     }
     return limit;
     } */
	void print(){
        std::cout << name << std::endl;
	}
};


class BCHand
{
    std::vector< BCDigit > BCDigits; //BCDigitPointer
    std::string name;
public:
    void print();
    
    
    
    
};
    
}; //From namespace barrett{}










#endif
