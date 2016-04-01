//
//  EPOSInterface.cpp
//  BairClawMain
//
//  Created by Randy Hellman on 10/14/13.
//  Copyright (c) 2013 Randy Hellman. All rights reserved.
//


#include "EPOSInterface.h"





namespace barrett {

void EPOS2::enable(){
    unsigned char data[8];
    int len =8;
    
    data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 00;
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X22; data[1] = 0X40; data[2] = 0X60; data[3] = 00;
    data[4] = 0X06; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 00;
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X22; data[1] = 0X40; data[2] = 0X60; data[3] = 00;
    data[4] = 0X0f; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 00;
    data[4] = 0X0f; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 00;
    data[4] = 0X0f; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X22; data[1] = 0X40; data[2] = 0X60; data[3] = 00;
    data[4] = 0X0f; data[5] = 0X01; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 00;
    data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
    bus->send(node, data, len);
    usleep(1000);
    isEnabled = true;
    
    return;
}

int EPOS2::getNode(){
    return node;
}
void EPOS2::reset(){
	unsigned char data[2];
	int len =2;
    
	data[0] = 0X81; data[1] = nodeSet;
	bus->send(0x000, data, len);
	btsleep(0.01);
	bus->send(0x000, data, len);
}

void EPOS2::reset(int node){
	unsigned char data[2];
	int len =2;
    
	data[0] = 0X81; data[1] = node;
	bus->send(0x000, data, len);
	btsleep(0.01);
	bus->send(0x000, data, len);
}

void EPOS2::resetAll(){
	unsigned char data[2];
	int len =2;
    
	data[0] = 0X81; data[1] = 0x00;
	bus->send(0x000, data, len);usleep(1000);
	bus->send(0x000, data, len);
}


void EPOS2::readAnalog(){
    
	readAnalog1();
    readAnalog2();

}
void EPOS2::readAnalog1(){
	unsigned char data[8];
	int len =8;
	data[0] = 0X40;
    data[1] = 0X7c;
    data[2] = 0X20;
    data[3] = 0x01;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
	bus->send(node, data, len);
	A1flag = 0;
    
}
void EPOS2::readAnalog2(){
	unsigned char data[8];
	int len =8;
	data[0] = 0X40;
    data[1] = 0X7c;
    data[2] = 0X20;
    data[3] = 0x02;
    data[4] = 0;
    data[5] = 0;
    data[6] = 0;
    data[7] = 0;
	bus->send(node, data, len);
	A2flag = 0;
}



void EPOS2::MoveToPosition(int pos, int controlWorld = 0){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	//Set target positon
	data[0] = 0x22;
	data[1] = 0x7a;
	data[2] = 0X60;
	data[3] = 0x00;
	data[4] = pos;
	data[5] = pos >> 8;
	data[6] = pos >> 16;
	data[7] = pos >> 24;
    
	bus->send(node, data, len);
	
    // Execute target postion either in abs or relative mode
    usleep(500);
	data[0] = 0x22;
	data[1] = 0x40;
	data[2] = 0x60;
	data[3] = 00;
	if 		( controlWorld == 0 ){
		data[4] = 0x3f; //absolute pos., start immediately
	}else if( controlWorld == 1){
		data[4] = 0x7f; //relative pos., start immediately
	}
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);

	
	return;
}
void EPOS2::GetMovementState(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
    
	data[0] = 0X40;
	data[1] = 0X41;
	data[2] = 0X60;
	data[3] = 00;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
    
	bus->send(node, data, len);btsleep(0.01);
}
void EPOS2::GetPosition(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
    
	data[0] = 0X40;
	data[1] = 0X64;
	data[2] = 0X60;
	data[3] = 0;
	data[4] = 0;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
    
	bus->send(node, data, len);btsleep(0.01);
}
void EPOS2::SetPositionProfile(int Velocity, int MaxAccel, int MaxDecel){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	int pos=0;
	data[0] = 0x22; data[1] = 0x81; data[2] = 0x60; data[3] = 00;
	pos = Velocity;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0x22; data[1] = 0x83; data[2] = 0x60; data[3] = 00;
	pos = MaxAccel;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0x22; data[1] = 0x84; data[2] = 0x60; data[3] = 00;
	pos = MaxDecel;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);btsleep(0.01);
    
	return;
}
void EPOS2::SetPositionRegulatorGain(int P, int I, int D){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	int pos=0;
	data[0] = 0X22; data[1] = 0Xfb; data[2] = 0X60; data[3] = 0X01;
	pos = P;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);
	btsleep(0.01);
	data[0] = 0X22; data[1] = 0Xfb; data[2] = 0X60; data[3] = 0X02;
	pos = I;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0Xfb; data[2] = 0X60; data[3] = 0X03;
	pos = D;
	data[4] = pos; data[5] = pos >> 8; data[6] = pos >> 16; data[7] = pos >> 24;
	bus->send(node, data, len);btsleep(0.01);
	return;
}

void EPOS2::DigitalOutputOn(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	data[0] = 0X40; data[1] = 0X41; data[2] = 0X60; data[3] = 0X00; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X79; data[2] = 0X20; data[3] = 0X01; data[4] = 0x0e; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X01; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X01; data[4] = 0x05; data[5] = 0x40; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X02; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X02; data[4] = 0x00; data[5] = 0x40; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X03; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X03; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	return;
}
void EPOS2::DigitalOutputOff(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	data[0] = 0X22; data[1] = 0X79; data[2] = 0X20; data[3] = 0X01; data[4] = 0x0e; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X01; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X01; data[4] = 0x05; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X02; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X02; data[4] = 0x00; data[5] = 0x40; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X40; data[1] = 0X78; data[2] = 0X20; data[3] = 0X03; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	data[0] = 0X22; data[1] = 0X78; data[2] = 0X20; data[3] = 0X03; data[4] = 0; data[5] = 0; data[6] = 0; data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
	return;
}
void EPOS2::ActivateProfilePositionMode(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
    
	data[0] = 0X22;
	data[1] = 0X60;
	data[2] = 0X60;
	data[3] = 0X00;
	data[4] = 0x01;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
    
	bus->send(node, data, len);btsleep(0.01);
    
	return;
}
void EPOS2::ActivateCurrentMode(int continuousCurrentLimit=5000, int outputCurrentLimit = 1000, int MaxSpeed = 25000){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;

    data[0] = 0X22;
	data[1] = 0X60;
	data[2] = 0X60;
	data[3] = 0X00;
	data[4] = 0xFD;
	data[5] = 0;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
    btsleep(0.01);
	data[0] = 0X22;
	data[1] = 0X10;
	data[2] = 0X64;
	data[3] = 0X01;
	data[4] = continuousCurrentLimit;
	data[5] = continuousCurrentLimit >> 8;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
    btsleep(0.01);
	
    
	data[0] = 0X22;
	data[1] = 0X10;
	data[2] = 0X64;
	data[3] = 0X04;
	data[4] = MaxSpeed;
	data[5] = MaxSpeed >> 8;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
    btsleep(0.01);
    
    data[0] = 0X22;
	data[1] = 0X10;
	data[2] = 0X64;
	data[3] = 0X02;
	data[4] = outputCurrentLimit;
	data[5] = outputCurrentLimit >> 8;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
    btsleep(0.01);
    
    

    
	return;
}
void EPOS2::SetCurrentLimit(int setCurrentLimit){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
	
	data[0] = 0X22;
	data[1] = 0X10;
	data[2] = 0X64;
	data[3] = 0X02;
	data[4] = setCurrentLimit;
	data[5] = setCurrentLimit >> 8;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);btsleep(0.01);
    
}
   
void EPOS2::SetMaxFollowingError(int MaxFollowingError){
    unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
    int len =8;
    
    data[0] = 0X22;
    data[1] = 0X65;
    data[2] = 0X60;
    data[3] = 0X00;
    data[4] = MaxFollowingError;
    data[5] = MaxFollowingError >> 8;
    data[6] = 0;
    data[7] = 0;
    bus->send(node, data, len);btsleep(0.01);
    
}

void EPOS2::SetCurrent(int setCurrent){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
    
	if(setCurrent > MAXCURRENTALLOWED)
    {
		setCurrent = MAXCURRENTALLOWED;
    }else if(setCurrent < -MAXCURRENTALLOWED)
    {
        setCurrent = -MAXCURRENTALLOWED;
    }
        
	currDemand = setCurrent;
	data[0] = 0X22;
	data[1] = 0X30;
	data[2] = 0X20;
	data[3] = 0X00;
	data[4] = setCurrent;
	data[5] = setCurrent >> 8;
	data[6] = 0;
	data[7] = 0;
	bus->send(node, data, len);

}

void EPOS2::DisableState(){
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	int len =8;
    
	data[0] = 0X22;
	data[1] = 0X40;
	data[2] = 0X60;
	data[3] = 00;
	data[4] = 0X00;
	data[5] = 0;
	data[6] = 0; 
	data[7] = 0;
    
	bus->send(node, data, len);btsleep(0.01);
    
	return;
}


    
} //From namespace barrett{}



