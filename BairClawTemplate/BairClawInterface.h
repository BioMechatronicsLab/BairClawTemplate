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

#include "EPOSInterface.h"




namespace barrett{

#pragma mark - BCDigit
class BCDigit {
	int setCounter;
	int JointShutOffRange;
public:
	EPOS2 FEmotor, PIPmotor, ADABmotor;
    int node;
	bool isInit;
	int adab, fe, pip, dip;
    int jointVal[4];
    double jointPercent[4];
	int ADABmin, FEmin, PIPmin, DIPmin;
	int ADABmax, FEmax, PIPmax, DIPmax;
	double ADABRange, FERange, PIPRange, DIPRange;
    double mcpFest, mcpEest, pipFest, pipEest, mcpFestOffset, mcpEestOffset, pipFestOffset, pipEestOffset;
                
	int normalRotation;                   //used to determine is flextion is pos mA. Set to neg(-) if flipped
	 //used to set static friction for each joint during initilization.
    std::string name;                     //name assined to BCdigit as a descrpiter
    
	// Constructor. Minimal error checking be carefule and make sure you
	// know what you are doing!!
	BCDigit(int node, const bus::CANSocket* busSet): FEmotor( node, busSet), PIPmotor( node+1, busSet), ADABmotor( node+2, busSet), node(node)
	{
		adab = 0; ADABmin = 18 ; ADABmax = 1020;
		fe   = 0; FEmin   = 480; FEmax   = 1021;
		pip  = 0; PIPmin  = 2  ; PIPmax  = 790;
		dip  = 0; DIPmin  = 611; DIPmax  = 832;
		setCounter = 0;
		isInit = 0;
		JointShutOffRange = 5;
		normalRotation = 0; //Default to normal rotation (+) mA for flextion
        
        //Initialize tendonForce Variables
        mcpFest = 0; mcpEest = 0; pipFest = 0; pipEest = 0;
        mcpFestOffset = 0; mcpEestOffset = 0; pipFestOffset = 0; pipEestOffset = 0;
        
	}
    
    /** \returns No return set jointVal[] property of class BCDigit
     *  \note neets to be called each time CAN dat frame is recieved to set data[8] to joint values
     */
	void set(unsigned char data[]){
		jointVal[0] = data[0] + (data[1] << 8);
		jointVal[1] = data[2] + (data[3] << 8);
		jointVal[2] = data[4] + (data[5] << 8);
		jointVal[3] = data[6] + (data[7] << 8);
	}
    void setTendonForceOffset();
    void calcTendonForce();
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


#pragma mark - BCHand
/**
 * \desc BCHand uses BCDigit to build a reference to complete hand. It also inherets from EPOS interface.
 */ 
class BCHand
{
public:
    std::vector<BCDigit> digit; //BCDigitPointer
    std::string name;

    BCHand(int NumberOfDigits, const bus::CANSocket* busSet, int statingNode=1)
    {
        digit.push_back(BCDigit(1, busSet));
    }
    void print(); //Displays bairclaw data on screen at ~10Hz not to be called from a realtime thread! 
    
};
    
    
    
    
}; //From namespace barrett{}



#endif
