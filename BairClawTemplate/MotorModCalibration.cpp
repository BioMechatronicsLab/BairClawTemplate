
/*=========================================================================
 | (c) 2013  Arizona State University Biomechatronics Lab
 |-----------------------------------------------------------------------
 | Project : Bair Claw Main/Interface
 | File    : MotorModCalibration.cpp
 | Authors : Randy Hellman (RBHellman@gmail.com, RHellman@asu.edu)
 |           Veronica Santos (Veronica.Santos@asu.edu)
 |-----------------------------------------------------------------------
 | Function: Function to collect data for motor module calibration multi 
 |           regression
 |-----------------------------------------------------------------------
 ========================================================================*/

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>
#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
//Boost martix allows easy matrix maniulation
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <unistd.h>
#include <barrett/os.h>  // For btsleep()
#include <boost/thread.hpp>
#include <barrett/bus/can_socket.h>
// Base class: barrett::systems::System
#include <barrett/systems/abstract/system.h>
#include <barrett/systems.h>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>
//Xenomia RTThread REQ
#include <native/task.h>
#include <native/timer.h>
#include <sys/mman.h>
//EPOS REQUIRED
#include "EPOSInterface.h"
#include "BairClawInterface.h"
#include "BairClawDataPlayBack.h"
//BioTac REQUIRED
//#include "cheetah.h"
//#include "biotac.h"
// UDP loadCell Server REQUIRED
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#define DEBUG true
#define POSITIONCONTROL false



using namespace barrett;
using namespace std;
using systems::connect;


RT_TASK can_receive_thread;

const int NUM_OF_DIGITS = 1;

const int BT_NUM_VAR_RECORD = 52;
const int CAN_PORT = 1; //0 or 1
const int NUM_JOINT_VAR_PASSED = 9;
bus::CANSocket CANbus(CAN_PORT);
static SRTIME delay=500000;

//Current Control Global ----//
int setCurrent = 0 ;
double static const Kp = 0.97;
double static const Kd = 0.01;
double static const Ki = 7.0;

double tensionDesired = 1500;
int node3currSetCount = 0, node3error = 0, A2setCounter = 0;
double error, integral, derivative, previous_error;


//---------------------------
bool going = true;
bool displayOn = false;
bool biotacInit = false;
bool shouldStart = false;

BCHand bairClaw(NUM_OF_DIGITS, &CANbus);
EPOS2 MCP_F(10, &CANbus), MCP_E(12, &CANbus), PIP_F(11, &CANbus), PIP_E(9, &CANbus);








#pragma mark - displayThread
void displayThread()
{
    int tick = 0;
    while(going)
    {
        while(going)
        {
            //NOT OK HERE NEED TO CLEANUP and correct the inital zero Analog readings
            bairClaw.digit[0].FEmotor.readAnalog1();
            rt_task_sleep(rt_timer_ns2ticks(delay));//
            bairClaw.digit[0].FEmotor.readAnalog2();
            rt_task_sleep(rt_timer_ns2ticks(delay));//
            bairClaw.digit[0].PIPmotor.readAnalog1();
            rt_task_sleep(rt_timer_ns2ticks(delay));//
            bairClaw.digit[0].PIPmotor.readAnalog2();
            
            
            if(displayOn)
            {
                system("clear");
                printf("Display- \n");
                printf("MCP_F - %d (mV), MCP_E - %d (mV), setCount - %d\n", bairClaw.digit[0].FEmotor.A2,bairClaw.digit[0].FEmotor.A1, A2setCounter/2);
                printf("PIP_F - %d (mV), PIP_E - %d (mV)\n", bairClaw.digit[0].PIPmotor.A2,bairClaw.digit[0].PIPmotor.A1);
                printf("SetCurrent - %d (mA), demandCurr - %d\n", setCurrent, bairClaw.digit[0].ADABmotor.currDemand);
                printf("error - %4.2f, int - %4.2f, deriv - %4.2f, pre-error - %4.2f \n", error, integral, derivative, previous_error);
                printf("FEset   - %d, error - %d\n", bairClaw.digit[0].FEmotor.currSetCount, bairClaw.digit[0].FEmotor.currSetError);
                printf("PIPset  - %d, error - %d\n", bairClaw.digit[0].PIPmotor.currSetCount, bairClaw.digit[0].PIPmotor.currSetError);
                printf("AdAbset - %d, error - %d\n", bairClaw.digit[0].ADABmotor.currSetCount, bairClaw.digit[0].ADABmotor.currSetError);
                /*printf("Digit - [%d, %d, %d, %d]\n",bairClaw.digit[0].jointVal[0], bairClaw.digit[0].jointVal[1], bairClaw.digit[0].jointVal[2], bairClaw.digit[0].jointVal[3]);
                bairClaw.digit[0].calcPercentage();
                printf("Digit - [%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].jointPercent[0], bairClaw.digit[0].jointPercent[1], bairClaw.digit[0].jointPercent[2], bairClaw.digit[0].jointPercent[3]);
                */
                printf("\nPress [Enter] to stop recording\n");
            
            }
            btsleep(0.1);
        }
        btsleep(1);
    }
    
}


#pragma mark - RTaskFunctions
RTIME secondsToRTIME(double s) {
	return static_cast<RTIME>(s * 1e9);
}

#pragma mark - UDP/IP communicaiton LoadCell read & BairClaw matlab visualizer

void err(char *str)
{
    perror(str);
    exit(1);
}

#pragma mark - canThread
double T_s_canBus = 0.0001;

void canReceiveThread(void *arg) {
    int ret;
	int id, SDO;
    EPOS2* currentEPOSController;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;
    
    rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s_canBus));
	RTIME now, previous;
    previous = rt_timer_read();
    int didReceiveNewDataFromFinger = 0;
    

    
    
    while (going)
    {
		rt_task_wait_period(NULL);
        ret  = CANbus.receiveRaw(id, data, len, false);
        
		
        
        if (ret == 0)
        {  // success read off CANbus not determine what to do with that message.
            SDO = data[1] + (data[2] << 8); // SERVICE DATA OBJECT (SDO)
            if (id == bairClaw.digit[0].FEmotor.nodeRec )
            {
                currentEPOSController = &bairClaw.digit[0].FEmotor;
            }
            else if (id == bairClaw.digit[0].PIPmotor.nodeRec )
            {
                currentEPOSController = &bairClaw.digit[0].PIPmotor;
            }
            else if (id == bairClaw.digit[0].ADABmotor.nodeRec )
            {
                currentEPOSController = &bairClaw.digit[0].ADABmotor;
            }
            
            if ( SDO == 0x207c ){
                if      (data[3] == 0x01)
                {
                    currentEPOSController->A1 = data[4] + (data[5] << 8);
                }
                else if (data[3] == 0x02)
                {
                    currentEPOSController->A2 = data[4] + (data[5] << 8);
                    A2setCounter++;
                }
            }
            else if ( SDO == 0x2030)
            {
                if(data[0] = 0x60)
                {
                    currentEPOSController->currSetCount++;
                }
                else if ( data[0] == 0x80)
                {
                    currentEPOSController->currSetError++;
                }
            }
		}
        else if (ret != 1)
        {  // error other than no data
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}
        
	}
    //                now = rt_timer_read();
    //                printf("Time since last turn: %ld.%06ld ms\n",(long)(now - previous) / 1000000, (long)(now - previous) % 1000000);
    //                previous = now;
}

void enableAll()
{
    PIP_E.enable();
    btsleep(0.25);
    PIP_F.enable();
    btsleep(0.25);
    MCP_E.enable();
    btsleep(0.25);
    MCP_F.enable();
    btsleep(0.25);
}

void setupCalibrationMotors()
{
    PIP_E.ActivateCurrentMode(5000, 5000, 10);
    PIP_E.SetCurrent(0);
    btsleep(0.1);
    PIP_F.ActivateCurrentMode(5000, 5000, 10);
    PIP_F.SetCurrent(0);
    btsleep(0.1);
    MCP_E.ActivateCurrentMode(5000, 5000, 10);
    MCP_E.SetCurrent(0);
    btsleep(0.1);
    MCP_F.ActivateCurrentMode(5000, 5000, 10);
    MCP_F.SetCurrent(0);
    btsleep(0.1);
}


#pragma mark - mainEntryPoint


int main(int argc, char** argv) {
    mlockall(MCL_CURRENT|MCL_FUTURE);/* Avoids memory swapping for this program */
    int mcpFcurr(0), mcpEcurr(0), pipFcurr(0), pipEcurr(0), maxRandomCurr(1000);
    double mcpFest(0), mcpEest(0), pipFest(0), pipEest(0);
    double mcpFestOffset(0), mcpEestOffset(0), pipFestOffset(0), pipEestOffset(0);
    
    //Log to file

    ofstream outH;
    outH.open("calibrationLog.txt", ios::trunc);
    if(!outH.good())
    {
        cout << "ERROR CREATING LOG FILE" << endl;
        exit(1);
    }

    boost::thread d(displayThread);
    //Creation and spinoff of RT_Threads
    rt_task_create(&can_receive_thread, "canReceive", 0, 51, 0);
    
    rt_task_start(&can_receive_thread, &canReceiveThread, NULL);
    

    going     = true;
    displayOn = false;
	enableAll();
    setupCalibrationMotors();
	   
    bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
    bairClaw.digit[0].FEmotor.enable();
    bairClaw.digit[0].ADABmotor.ActivateProfilePositionMode();
    bairClaw.digit[0].ADABmotor.enable();
    bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
    bairClaw.digit[0].PIPmotor.enable();

    
    
    
    // SetOffset for SG values
    system("clear");
    printf("\nInitializing Strain Gages.");
    fflush(stdout);
    for(int i=0; i<10; i++)
    {
        mcpFestOffset = -5.971124e+02 + (+8.402597e-01 * bairClaw.digit[0].FEmotor.A2) + (-4.617917e-02 * bairClaw.digit[0].FEmotor.A1) + (-1.800978e-02 * bairClaw.digit[0].PIPmotor.A2) + (-5.823183e-02 * bairClaw.digit[0].PIPmotor.A1);
        mcpEestOffset = -1.753974e+03 + (-1.241008e-02 * bairClaw.digit[0].FEmotor.A2) + (+2.670576e+00 * bairClaw.digit[0].FEmotor.A1) + (-3.922166e-02 * bairClaw.digit[0].PIPmotor.A2) + (-1.786393e-01 * bairClaw.digit[0].PIPmotor.A1);
        pipFestOffset = -8.943469e+02 + (-3.292929e-02 * bairClaw.digit[0].FEmotor.A2) + (-2.342654e-02 * bairClaw.digit[0].FEmotor.A1) + (+9.952569e-01 * bairClaw.digit[0].PIPmotor.A2) + (-5.316950e-02 * bairClaw.digit[0].PIPmotor.A1);
        pipEestOffset = -2.949234e+03 + (-9.191377e-02 * bairClaw.digit[0].FEmotor.A2) + (-1.433349e-01 * bairClaw.digit[0].FEmotor.A1) + (+8.367845e-02 * bairClaw.digit[0].PIPmotor.A2) + (+3.056697e+00 * bairClaw.digit[0].PIPmotor.A1);
        btsleep(0.2);
        printf(".");
        fflush(stdout);
    }
    
    
    for(int j=0; j<500; j++)
    {
        mcpFcurr = rand() % maxRandomCurr;
        mcpEcurr = rand() % maxRandomCurr;
        pipFcurr = rand() % maxRandomCurr;
        pipEcurr = rand() % maxRandomCurr;
        /*
        btsleep(0.05);
        PIP_E.SetCurrent(-pipEcurr);
        btsleep(0.1);
        PIP_F.SetCurrent(-pipFcurr);
        btsleep(0.1);
        MCP_E.SetCurrent(mcpEcurr);
        btsleep(0.1);
        MCP_F.SetCurrent(mcpFcurr);
        */
        btsleep(0.05);
        system("clear");
        //outH << j << ", " << bairClaw.digit[0].FEmotor.A2 << ", " << bairClaw.digit[0].FEmotor.A1 << ", " << bairClaw.digit[0].PIPmotor.A2 << ", " << bairClaw.digit[0].PIPmotor.A1 << ", " << mcpFcurr << ", " << mcpEcurr << ", " << pipFcurr << ", " << pipEcurr << endl;
        //cout << j << ", " <<  bairClaw.digit[0].FEmotor.A2 << ", " << bairClaw.digit[0].FEmotor.A1 << ", " << bairClaw.digit[0].PIPmotor.A2 << ", " << bairClaw.digit[0].PIPmotor.A1 << ", " << mcpFcurr << ", " << mcpEcurr << ", " << pipFcurr << ", " << pipEcurr << endl;
        
        
        mcpFest = (-5.971124e+02 + (+8.402597e-01 * bairClaw.digit[0].FEmotor.A2) + (-4.617917e-02 * bairClaw.digit[0].FEmotor.A1) + (-1.800978e-02 * bairClaw.digit[0].PIPmotor.A2) + (-5.823183e-02 * bairClaw.digit[0].PIPmotor.A1)) - (mcpFestOffset);
        mcpEest = (-1.753974e+03 + (-1.241008e-02 * bairClaw.digit[0].FEmotor.A2) + (+2.670576e+00 * bairClaw.digit[0].FEmotor.A1) + (-3.922166e-02 * bairClaw.digit[0].PIPmotor.A2) + (-1.786393e-01 * bairClaw.digit[0].PIPmotor.A1)) - (mcpEestOffset);
        pipFest = (-8.943469e+02 + (-3.292929e-02 * bairClaw.digit[0].FEmotor.A2) + (-2.342654e-02 * bairClaw.digit[0].FEmotor.A1) + (+9.952569e-01 * bairClaw.digit[0].PIPmotor.A2) + (-5.316950e-02 * bairClaw.digit[0].PIPmotor.A1)) - (pipFestOffset);
        pipEest = (-2.949234e+03 + (-9.191377e-02 * bairClaw.digit[0].FEmotor.A2) + (-1.433349e-01 * bairClaw.digit[0].FEmotor.A1) + (+8.367845e-02 * bairClaw.digit[0].PIPmotor.A2) + (+3.056697e+00 * bairClaw.digit[0].PIPmotor.A1)) - (pipEestOffset);
        
        printf("mcpFest - %+6.2f\n", mcpFest);
        printf("mcpEest - %+6.2f\n", mcpEest);
        printf("pipFest - %+6.2f\n", pipFest);
        printf("pipEest - %+6.2f\n", pipEest);
        printf("  count = \n",j);


    }

    outH.close();
    




    
    bairClaw.digit[0].FEmotor.resetAll();
    displayOn = false; // has display thread clear and print to screen
    going = false;
    d.join(); //Stop display thread
    bairClaw.digit[0].FEmotor.resetAll();
    cout << " join d" << endl;


    rt_task_delete(&can_receive_thread);
    cout << " can_receive_returned" << endl;
    
    
    
    
    

    
    
    
	
    
	return 0;
}
