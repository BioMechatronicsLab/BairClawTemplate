/*=========================================================================
 | (c) 2013  Arizona State University Biomechatronics Lab
 |--------------------------------------------------------------------------
 | Project : Bair Claw Main/Interface
 | File    : BairClawMain.cpp
 | Authors : Randy Hellman (RBHellman@gmail.com, RHellman@asu.edu)
 |           Veronica Santos (Veronica.Santos@asu.edu)
 |--------------------------------------------------------------------------
 | Function: Main entry point for BairClaw controls CAN communication between
 | 			BairClaw and EPOS motor controllers. Adapted from ex05/sg_log
 |           and etc...
 |--------------------------------------------------------------------------
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
#include <ctime>
#include <time.h>

#include <boost/thread.hpp>
#include <boost/thread/condition_variable.hpp>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
//Boost martix allows easy matrix maniulation
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <unistd.h>
#include <barrett/os.h>  // For btsleep()
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
#include "cheetah.h"
#include "BCbiotac.h"
#include "filters.h"
// UDP loadCell Server REQUIRED
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/types.h>
#include <sys/socket.h>

#define DEBUG true
#define POSITIONCONTROL true
#define biotacOff false


using namespace barrett;
using systems::connect;

RT_TASK bt_record_thread;
RT_TASK can_receive_thread;
RT_TASK rt_control_thread; //This is going to directly send CAN communcations to set MoveToPosition

const int NUM_OF_DIGITS = 1;

const int BT_NUM_VAR_RECORD = 52;
const int CAN_PORT = 1; //0 or 1
const int NUM_JOINT_VAR_PASSED = 9;
const int loggerOffset = 45;
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
//PlayBackData
dataRecording *playbackData = new dataRecording();
//Matlab RT visualizer
std::string host;
//global state machine variables
enum state{positionControl, pdcControl, idle};
state currentState = positionControl;
int internalState = 0;
//---------------------------
boost::condition_variable cv;
boost::mutex mut;
bool ready = false;

bool going = true;
bool displayOn = false;
bool biotacInit = false;
bool shouldStart = false;
bt_single_batch data;
BCHand bairClaw(NUM_OF_DIGITS, &CANbus);
struct lCell
{
    double x,   y,  z;
    double tx, ty, tz;
    int numOfMsgRevieved;
}loadCell;



math::Vector< BT_NUM_VAR_RECORD >::type btInput;
systems::ExposedOutput<math::Vector< NUM_JOINT_VAR_PASSED >::type> eoSys;
systems::ExposedOutput<math::Vector< BT_NUM_VAR_RECORD >::type> btEoSys;
math::Vector< NUM_JOINT_VAR_PASSED >::type jointInput;



#pragma mark - BCProductManager
class BCProductManager : public systems::System {
    // IO
    // Marked as "public" because Inputs and Output are (except in special cases)
    // part of a System's public interface.
public:
    Input<math::Vector< NUM_JOINT_VAR_PASSED >::type> input;
    Output<int> output;
    Output<math::Vector< NUM_JOINT_VAR_PASSED >::type> jointsOutput;
    
    int countCheck;

    
    // Marked as "protected" because this object lets us change the value of an
    // Output, which should only be allowed from within this System.
protected:
    Output<int>  ::Value* outputValue;
    Output<math::Vector< NUM_JOINT_VAR_PASSED >::type>::Value* jointsOutputValue;
    math::Vector< NUM_JOINT_VAR_PASSED >::type joints;


public:
	// Every System has a human readable name. It's good practice to provide an
	// appropriate default. Notice that outputValue is associated with output
	// via output's constructor.
	explicit BCProductManager(const std::string& sysName = "BairClawProductManager") : systems::System(sysName), input(this), output(this, &outputValue),jointsOutput(this, &jointsOutputValue),  countCheck(0)
    {
        error = 0;
        integral = 0;
        derivative = 0;
        previous_error = 0;
    }
    
	// Every System is required to call System::mandatoryCleanUp() in its
	// destructor, preferably as early as possible. It's common for libbarrett
	// to be used in a multi-threaded environment. This function cleans up all
	// of libbarrett's references to this System so that the library won't try
	// to interact with it from Thread A while it's in the process of being
	// destroyed in Thread B. If you forget this, you may occasionally see your
	// program crash with the message: "Pure virtual function called".
	virtual ~BCProductManager() { mandatoryCleanUp(); }
    
    int getCount()
    {
        return countCheck;
    }
protected:
    
    math::Vector< NUM_JOINT_VAR_PASSED > ::type jointActual;
	// Implement System::operate(). The operate() function must be declared with
	// the "protected" access specifier.
	virtual void operate() {
		jointActual = input.getValue();  // Pull data from the input
        
        
        static int count = 0;
        count++;
        countCheck++;
        
        for( int j(0); j< NUM_JOINT_VAR_PASSED; j++)
        {
            joints[j] = jointActual[j];
        }
        
        if (shouldStart == true)
        {
            tensionDesired = playbackData->getDataValueAtIndex(count % (playbackData->dataCount-2));
        }
        else
        {
            tensionDesired = 1500;
        }

        
        
        //Initial setup for PID to control current on Flexion of MCP
        error = tensionDesired - (double)bairClaw.digit[0].FEmotor.A2;
        integral = integral + error * 0.005;
        derivative = (error - previous_error) / 0.005;
        if(!DEBUG){
            setCurrent = (int) ( (Kp * error) + (Ki * integral) + (Kd * derivative) );
        }
        else{
            setCurrent = 0;
        }
        previous_error = error;
        
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        //Should be moved into SetCurrent to account for static friciton///////////////////////////
        if(setCurrent < 0)/////////////////////////////////////////////////////////////////////////
        {
            setCurrent =   abs(setCurrent) + abs(bairClaw.digit[0].FEmotor.staticFrictionE);
        }else if(setCurrent > 0)
        {
            setCurrent =   -(setCurrent + abs(bairClaw.digit[0].FEmotor.staticFrictionF));
        }else
        {
            setCurrent = 0;
        }//////////////////////////////////////////////////////////////////////////////////////////
        bairClaw.digit[0].FEmotor.SetCurrent( setCurrent);
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.SetCurrent(0);
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].ADABmotor.SetCurrent(setCurrent);//--------------------------
        rt_task_sleep(rt_timer_ns2ticks(delay));//

        //REQUESTS EPOS analog values at same freq as BCProductManager
        bairClaw.digit[0].FEmotor.readAnalog1();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].FEmotor.readAnalog2();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.readAnalog1();
        rt_task_sleep(rt_timer_ns2ticks(delay));//
        bairClaw.digit[0].PIPmotor.readAnalog2();
        
        outputValue->setData(&count);
        jointsOutputValue->setData(&joints);
        

        
	}
    
    
};
#pragma mark - displayThread
void displayThread()
{
    int tick = 0;
    while(going)
    {
        while(displayOn && going)
        {
            system("clear");
            printf("Display- \n");
            printf("MCP_F - %d (mV), MCP_E - %d (mV), setCount - %d\n", bairClaw.digit[0].FEmotor.A2,bairClaw.digit[0].FEmotor.A1, A2setCounter/2);
            printf("PIP_F - %d (mV), PIP_E - %d (mV), setCount - %d\n", bairClaw.digit[0].PIPmotor.A2,bairClaw.digit[0].PIPmotor.A1);
            printf("SetCurrent - %d (mA), demandCurr - %d\n", setCurrent, bairClaw.digit[0].ADABmotor.currDemand);
            printf("error - %4.2f, int - %4.2f, deriv - %4.2f, pre-error - %4.2f \n", error, integral, derivative, previous_error);
            printf("FEset   - %d, error - %d\n", bairClaw.digit[0].FEmotor.currSetCount, bairClaw.digit[0].FEmotor.currSetError);
            printf("PIPset  - %d, error - %d\n", bairClaw.digit[0].PIPmotor.currSetCount, bairClaw.digit[0].PIPmotor.currSetError);
            printf("AdAbset - %d, error - %d\n", bairClaw.digit[0].ADABmotor.currSetCount, bairClaw.digit[0].ADABmotor.currSetError);
            printf("Digit - [%d, %d, %d, %d]\n",bairClaw.digit[0].jointVal[0], bairClaw.digit[0].jointVal[1], bairClaw.digit[0].jointVal[2], bairClaw.digit[0].jointVal[3]);
           
            
            printf("Digit JointPercent-\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].jointPercent[0], bairClaw.digit[0].jointPercent[1], bairClaw.digit[0].jointPercent[2], bairClaw.digit[0].jointPercent[3]);
            printf("Digit JointAngleRad  -\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].jointValRad[0], bairClaw.digit[0].jointValRad[1], bairClaw.digit[0].jointValRad[2], bairClaw.digit[0].jointValRad[3]);
            printf("Digit TendonForce -\n[%4.2f, %4.2f, %4.2f, %4.2f]\n",bairClaw.digit[0].mcpFest, bairClaw.digit[0].mcpEest, bairClaw.digit[0].pipFest, bairClaw.digit[0].pipEest);
            /*
            std::cout << "Jacobian" << std::endl << std::endl << std::endl;
            std::cout << std::setprecision(5) << std::fixed << bairClaw.digit[0].DHp.jacobian << std::endl;
            std::cout << "jacobianTransposePseudoInverse()" << std::endl << std::endl;
            std::cout << bairClaw.digit[0].DHp.jacobianTransposePseudoInverse << std::endl;
            std::cout << "jacobianActuation" << std::endl;
            std::cout << bairClaw.digit[0].DHp.jacobianActuation << std::endl;
            */
            std::cout << "loadCell" << std::endl;
            printf("[%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f]\n",loadCell.x, loadCell.y, loadCell.z, loadCell.tx, loadCell.ty, loadCell.tz);
            std::cout << std::endl << "endEffectorForce" << std::endl;
            
            
            printf("\nP_dc - %4.2f\n", btInput[1]);
            printf(" currentState  - %d\n", currentState);
            printf(" internalState - %d\n", internalState);
            printf("\nPress [Enter] to stop recording\n");
            
            btsleep(0.1);
        }
        btsleep(0.5);
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



void loadcellRecordThread()
{
    const int PORT = 12345;
    const int BUFLEN = 1024;
    
    struct sockaddr_in my_addr, cli_addr;
    int sockfd, i;
    socklen_t slen=sizeof(cli_addr);
    char buf[BUFLEN];
    
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
    
    loadCell.numOfMsgRevieved  = 0;

    
    while (going)
    {
        if (recvfrom(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1)
            err("recvfrom()");
        
        sscanf(buf,"%lf %lf %lf %lf %lf %lf", &(loadCell.x), &(loadCell.y), &(loadCell.z), &(loadCell.tx), &(loadCell.ty), &(loadCell.tz) );
//        printf("Received packet from %s:%d\nData: %s",
//               inet_ntoa(cli_addr.sin_addr), ntohs(cli_addr.sin_port), buf);
//        printf("counter = %d\n",loadCell.numOfMsgRevieved++);
//
//        printf("loadCell Values [ %6.3f, %6.3f, %6.3f, %6.3f, %6.3f, %6.3f]\n\n", loadCell.x, loadCell.y, loadCell.z, loadCell.tx, loadCell.ty, loadCell.tz);
        
        
        btInput[loggerOffset]     = tensionDesired;
        btInput[loggerOffset + 1] = loadCell.x;
        btInput[loggerOffset + 2] = loadCell.y;
        btInput[loggerOffset + 3] = loadCell.z;
        btInput[loggerOffset + 4] = loadCell.tx;
        btInput[loggerOffset + 5] = loadCell.ty;
        btInput[loggerOffset + 6] = loadCell.tz;
        
    }
    
    close(sockfd);
    
}

void bairClawMatlabVisualizerSender()
{
    printf("In bairClaw matlab vis");
    const int BUFLEN = 4096;
    const int PORT = 5678;
    
    
    struct sockaddr_in serv_addr;
    int sockfd, i, slen=sizeof(serv_addr);
    char buf[BUFLEN];
    
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1)
        err("socket");
    
    bzero(&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(PORT);
    if (inet_aton(host.c_str(), &serv_addr.sin_addr)==0)
    {
        fprintf(stderr, "inet_aton() failed\n");
        exit(1);
    }
    int sendCnt = 0;
    while(going)
    {
        memset(buf, 0, BUFLEN);
        //SEND(jointInput,Tendontension,biotacFULL)
        for(int i=0; i<4; i++) //
        {
            sprintf(buf + strlen(buf),"%d, ",bairClaw.digit[0].jointVal[i]);
        }
        sprintf(buf + strlen(buf),"%d, %d, %d, %d, ", bairClaw.digit[0].PIPmotor.A1, bairClaw.digit[0].PIPmotor.A2, bairClaw.digit[0].FEmotor.A1, bairClaw.digit[0].FEmotor.A2);
        for(int i=0; i<46; i++)
        {
            sprintf(buf + strlen(buf),"%d, ", int(btInput[i]));
        }
        for(int i=46; i<52; i++)
        {
            sprintf(buf + strlen(buf),"%4.4f, ", double(btInput[i]));
        }
        sprintf(buf + strlen(buf), "\n");

        if(strcmp(buf,"exit") == 0)
            exit(0);
        
        if (sendto(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&serv_addr, slen)==-1)
            err("sendto()");
        btsleep(0.1);
    }
    //exit matlabloop
    memset(buf, 0, BUFLEN);
    sprintf(buf,"EXIT");
    for(int i=0; i<10;i++)
    {
        if (sendto(sockfd, buf, BUFLEN, 0, (struct sockaddr*)&serv_addr, slen)==-1)
            err("sendto()");
        btsleep(0.05);
    }
    close(sockfd);
}

#pragma mark - BioTac Record Thread
double T_s_biotac = 0.01;
int x = 0 ;

void biotacRecordThread(void *arg){
    /****************************/
	/* --- Define variables --- */
	/****************************/
    bt_info biotac;
    bt_property biotac_property[MAX_BIOTACS_PER_CHEETAH];
	//bt_single_batch data;   DEFINED GLOBALLY!
	BioTac bt_err_code;
	Cheetah ch_handle;
	int i;
	double length_of_data_in_second;
	int number_of_samples;
	int number_of_loops;
    
    if(!biotacOff)
    {
        /**************************************************************************/
        /* --- Initialize BioTac settings (only default values are supported) --- */
        /**************************************************************************/
        biotac.spi_clock_speed = BT_SPI_BITRATE_KHZ_DEFAULT;
        biotac.number_of_biotacs = 0;
        biotac.sample_rate_Hz = BT_SAMPLE_RATE_HZ_DEFAULT;
        biotac.frame.frame_type = 0;
        biotac.batch.batch_frame_count = 1; //BT_FRAMES_IN_BATCH_DEFAULT;
        biotac.batch.batch_ms = 10; //BT_BATCH_MS_DEFAULT;
        //had to change to get single batch of BioTac data (44 samples at 100Hz)

        
        // Set the duration of the run time
        length_of_data_in_second = 0.01;
        number_of_samples = (int)(BT_SAMPLE_RATE_HZ_DEFAULT * length_of_data_in_second);
        std::cout << " number_of_samples - " << number_of_samples << std::endl;
        
        // Check if any initial settings are wrong
        if (MAX_BIOTACS_PER_CHEETAH != 3 && MAX_BIOTACS_PER_CHEETAH != 5)
        {
            bt_err_code = BT_WRONG_MAX_BIOTAC_NUMBER;
            bt_display_errors(bt_err_code);
            exit(1);
        }
        /******************************************/
        /* --- Initialize the Cheetah devices --- */
        /******************************************/
        ch_handle = bt_cheetah_initialize(&biotac);
        
        /*********************************************************/
        /* --- Get and print out properties of the BioTac(s) --- */
        /*********************************************************/
        
        for (i = 0; i < MAX_BIOTACS_PER_CHEETAH; i++)
        {
            bt_err_code = bt_cheetah_get_properties(ch_handle, i+1, &(biotac_property[i]));
            
            if (biotac_property[i].bt_connected == YES)
            {
                (biotac.number_of_biotacs)++;
            }
            
            if (bt_err_code)
            {
                bt_display_errors(bt_err_code);
                exit(1);
            }
        }
        
        // Check if any BioTacs are detected
        if (biotac.number_of_biotacs == 0)
        {
            bt_err_code = BT_NO_BIOTAC_DETECTED;
            bt_display_errors(bt_err_code);
        }
        else
        {
            printf("\n%d BioTac(s) detected.\n\n", biotac.number_of_biotacs);
        }

        
        /*******************************/
        /* --- Configure the batch --- */
        /*******************************/
        bt_err_code = bt_cheetah_configure_batch(ch_handle, &biotac, number_of_samples);
        if (bt_err_code < 0)
        {
            bt_display_errors(bt_err_code);
            exit(1);
        }
        else
        {
            printf("\nConfigured the batch\n");
        }
        
        printf("about to collect \n");
        number_of_loops = (int)(number_of_samples / ((double)(biotac.frame.frame_size * biotac.batch.batch_frame_count)));
        printf("Start collecting BioTac data for %4.0f seconds(s)\n\tnumber_of_loops - %d \n", length_of_data_in_second,number_of_loops);
        
        printf("Press [Enter] to continue ...");
        fflush(stdout);
        getchar();
        
        biotacInit = true; //Flag to tell main that biotac is init.
        

        rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s_biotac));
        while(!shouldStart)
        {
            rt_task_wait_period(NULL);
        }
        RTIME now, previous;
        
        
        
        
        previous = rt_timer_read();
        while (going)
        {
            rt_task_wait_period(NULL);
        
            bt_cheetah_collect_single_batch(ch_handle, &biotac, &data, NO);
            
            btInput[0] = x++; //Counter to ensure fresh batch of data
            btInput[1] = data.bt[0].pdc;
            btInput[2] = data.bt[0].tac;
            btInput[3] = data.bt[0].tdc;
            
            for(i=0; i<19; i++) //Set electrode values into logget
            {
                btInput[i+4]= data.bt[0].elec[i];
            }
            
            for(i=0; i<22; i++) //Set pac into a singe row to be pulled out
            {
                btInput[i+23] = data.bt[0].pac[i];
            }
            
            btEoSys.setValue(btInput);
        }

    }
}

#pragma mark - canThread
double T_s_canBus = 0.0001;

void canReceiveThread(void *arg) {
    int ret;
	int id, SDO, i;
    EPOS2* currentEPOSController;
	unsigned char data[bus::CANSocket::MAX_MESSAGE_LEN];
	size_t len;
    
    rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s_canBus));
	RTIME now, previous;
    previous = rt_timer_read();
    int didReceiveNewDataFromFinger = 0;

    //NOT OK HERE NEED TO CLEANUP and correct the inital zero Analog readings
    bairClaw.digit[0].FEmotor.readAnalog1();
    rt_task_sleep(rt_timer_ns2ticks(delay));//
    bairClaw.digit[0].FEmotor.readAnalog2();
    rt_task_sleep(rt_timer_ns2ticks(delay));//
    bairClaw.digit[0].PIPmotor.readAnalog1();
    rt_task_sleep(rt_timer_ns2ticks(delay));//
    bairClaw.digit[0].PIPmotor.readAnalog2();
    
    while (going)
    {
		rt_task_wait_period(NULL);
        ret  = CANbus.receiveRaw(id, data, len, false);
        
		if (ret == 0)
        {  // success read off CANbus not determine what to do with that message.

            if(id == 0x201) //CANid of index finger set on arduino/CANbusShield.
            {
                bairClaw.digit[0].set(data);

                jointInput[0] = didReceiveNewDataFromFinger++;
                for(i=1; i<5; i++)
                {
                    jointInput[i] = bairClaw.digit[0].jointVal[i-1];
                }
                
                jointInput[5] = bairClaw.digit[0].FEmotor.A1;
                jointInput[6] = bairClaw.digit[0].FEmotor.A2;
                jointInput[7] = bairClaw.digit[0].PIPmotor.A1;
                jointInput[8] = bairClaw.digit[0].PIPmotor.A2;
                
                eoSys.setValue(jointInput);
            }else
            {
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
                        jointInput[5] = bairClaw.digit[0].FEmotor.A1;
                        jointInput[6] = bairClaw.digit[0].FEmotor.A2;
                        jointInput[7] = bairClaw.digit[0].PIPmotor.A1;
                        jointInput[8] = bairClaw.digit[0].PIPmotor.A2;
                        
                        eoSys.setValue(jointInput);
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

		} else if (ret != 1) {  // error other than no data
			printf("ERROR: bus::CANSocket::receive() returned %d.\n", ret);
		}
        
	}
//                now = rt_timer_read();
//                printf("Time since last turn: %ld.%06ld ms\n",(long)(now - previous) / 1000000, (long)(now - previous) % 1000000);
//                previous = now;
}

//RT_CONTROL_LOOP - this loop is used for maintain a desired finger tapping freq.
#pragma mark - rt_control_loop


double T_s_rtControlThreadDelay = 0.01;
void rtControlThread(void *arg){
    
    rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s_rtControlThreadDelay));
    std::cout << "cv.wait(lock)" << std::endl;
    boost::unique_lock<boost::mutex> lock(mut);
    double desiredPos = 5, desiredPosPIPDIP = 10, desiredPdc = 2000, pdcError;
    double desiredPdcPIPDIPmaxJointPercentage = 40;
    
    
    double changeInMotorPosFE = 0, changeInMotorPosPD = 0;
    int sw=0, count=0;
    //enum currentState{positionControl, pdcControl, idle}; DEFINED GLOBALLY
    state previousState = idle; //ensures that previousState and currentState start differently
    //Define biotac filter
    double numBW[] = {   0.002898,   0.008695,   0.008695,   0.002898};
    double denBW[] = {   1.000000,  -2.374095,   1.929356,  -0.532075};
    butterworthFilter btpdcFilter(3, numBW, denBW);
    double btPdcInit = 0;
    if(POSITIONCONTROL)
    {
        bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].FEmotor.SetMaxFollowingError(MAX_FOLLOWING_ERROR);
        bairClaw.digit[0].FEmotor.enable();
        bairClaw.digit[0].ADABmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].ADABmotor.enable();
        bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
        bairClaw.digit[0].PIPmotor.SetMaxFollowingError(MAX_FOLLOWING_ERROR);
        bairClaw.digit[0].PIPmotor.enable();
    }
    
    while( !ready)
    { //waits until main is ready to start running thread. this allows everything to initialize properly.
        cv.wait(lock);
        rt_task_wait_period(NULL);
    }
    btPdcInit = btInput[1];
    while (true)
    {
        rt_task_wait_period(NULL);
        
         bairClaw.digit[0].calcPercentage();
         bairClaw.digit[0].calcJointAngles();
         bairClaw.digit[0].calcTendonForce();
        /* //UNCOMMENT TO ADD BACK IN END EFFECTOR FORCE CALCS
         bairClaw.digit[0].calcJacobianActuation();
         bairClaw.digit[0].calcDHparams(); //Will also clac jacobian, pinv(jacobian')
         bairClaw.digit[0].calcEndEffectorForce(); //end effoctor force
         
         btInput[loggerOffset + 7]  = bairClaw.digit[0].DHp.endEffectorForce(0,0);
         btInput[loggerOffset + 8]  = bairClaw.digit[0].DHp.endEffectorForce(1,0);
         btInput[loggerOffset + 9]  = bairClaw.digit[0].DHp.endEffectorForce(2,0);
         btInput[loggerOffset + 10] = bairClaw.digit[0].DHp.endEffectorForce(3,0);
         btInput[loggerOffset + 11] = bairClaw.digit[0].DHp.endEffectorForce(4,0);
         btInput[loggerOffset + 12] = bairClaw.digit[0].DHp.endEffectorForce(5,0);
         */
        btpdcFilter.update(btInput[1]);

        switch(currentState)
        {
            /**
             * \desc Position control loop to move the hand to a desired location unless contact is made then it will switch modes.
             */
            case positionControl:
                
                if(previousState != currentState)
                {
                    bairClaw.digit[0].FEmotor.SetCurrentLimit(350);
                    bairClaw.digit[0].FEmotor.SetPositionProfile(50,2500,2500);
                    bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
                    
                    bairClaw.digit[0].PIPmotor.SetCurrentLimit(350);
                    bairClaw.digit[0].PIPmotor.SetPositionProfile(50,2500,2500);
                    bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
                    internalState = 1;
                }
                previousState = currentState;
                
                changeInMotorPosFE = (bairClaw.digit[0].jointPercent[1] - desiredPos) * 50;
                changeInMotorPosPD = ((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) - desiredPosPIPDIP) * 50;
                if(count % 2 == 0)
                {
                    bairClaw.digit[0].FEmotor.MoveToPosition(changeInMotorPosFE, 1);
                }
                else
                {
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                }

                
                desiredPos = 50;
                desiredPosPIPDIP = 10; //WAS 10
                
                if(btInput[1] > btPdcInit+20)
                {
                    currentState = pdcControl;
                }
                /*
                if(count % 100 == 0){
                    sw++;
                    if(sw % 2 == 0 ){
                        desiredPos = 17;
                        desiredPosPIPDIP = 12; //WAS 70
                    }else{
                        desiredPos = 0;
                        desiredPosPIPDIP = 10; //WAS 10
                    }
                }*/

                
                break;
            /**
             * \desc pdcControl controlls motor torques to maintain a desired pdc.
             */
            case pdcControl:
               
                
                if(previousState != currentState)
                {
                    count = 0;
                    bairClaw.digit[0].FEmotor.SetCurrentLimit(900);
                    bairClaw.digit[0].FEmotor.SetPositionProfile(500,2500,2500);
                    bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();
                    
                    bairClaw.digit[0].PIPmotor.SetCurrentLimit(400);
                    bairClaw.digit[0].PIPmotor.SetPositionProfile(500,2500,2500);
                    bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();
                    internalState = 1;
                    
                }
                previousState = currentState;
                
                if(count < 300)
                {
                    pdcError = desiredPdc - btInput[1];
                    changeInMotorPosFE = -pdcError;
                    
                    if((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) < desiredPdcPIPDIPmaxJointPercentage)
                    {
                        changeInMotorPosPD = -pdcError/10;
                    }
                    else
                    {
                        changeInMotorPosPD = 0;
                    }
                    bairClaw.digit[0].FEmotor.MoveToPosition (changeInMotorPosFE, 1);
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                }
                else if (count < 500){
                    if(internalState < 2)
                    {
                        desiredPos = 2;
                        desiredPosPIPDIP = 3;
                        bairClaw.digit[0].FEmotor.SetPositionProfile(300,2500,2500);
                        bairClaw.digit[0].PIPmotor.SetPositionProfile(100,2500,2500);
                        internalState = 2;
                    }
                    internalState = 3;
                    changeInMotorPosFE = (bairClaw.digit[0].jointPercent[1] - desiredPos) * 50;
                    changeInMotorPosPD = ((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) - desiredPosPIPDIP) * 50;
                    
                    bairClaw.digit[0].FEmotor.MoveToPosition(changeInMotorPosFE, 1);
                    bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
                    
                }
                else
                {
                    currentState = positionControl;
                    internalState = 1;
                }
                
                

                break;
            //-------------------------------------------------------------------------------//
            case idle:
                previousState = currentState;
                break;
        }
        count++;
    }
    

    
}


#pragma mark - mainEntryPoint


int main(int argc, char** argv) {
    mlockall(MCL_CURRENT|MCL_FUTURE);/* Avoids memory swapping for this program */
    
    /*//------------------ FHN MERGER SUCCESS?
    DHparams DHp;
    // Jacobian and Matrix test---------------------------------------------------------
    DHp.calcT();
    int runNum = 1000000;
    VectorXd thetaUpdate(4);
    clock_t begin = clock();
    for(int i=0; i < runNum; i++)
    {
        thetaUpdate << 0, 1, 0, 1;
        DHp.calcT(thetaUpdate);
        if( i%10000 == 0)
            std::cout << (double(i)/double(runNum))*100 << "%" << std::endl;
        DHp.calcJacobian();
        DHp.pinvJacobian();
        
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    
    std::cout << "elapsed_secs = " << elapsed_secs << std::endl;
    std::cout << "Hz = " << double(runNum)/elapsed_secs << std::endl;
    
    std::cout << "Jacobian" << std::endl << std::endl;
    std::cout << DHp.jacobian << std::endl;
    std::cout << std::endl << "pinv(Jacobian) " << std::endl;
    std::cout << DHp.jacobianPseudoInverse << std::endl;
    //-----------------------------------------------------------------------------------*/
    
    //make temp logging file info
    char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1)
    {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
    //Load in playbackData
    playbackData->readFile("data2Track.bin");
    if(argc >= 2)
    {
        host = argv[1];
        printf("Visualizer data being sent to ip: %s \n",argv[1]);
        
    }else{
        host = "127.0.0.1";
    }

    //Creation and spinoff of RT_Threads (creates hard realtime thread)
    rt_task_create(&bt_record_thread, "btRecord", 0, 50, 0);
    rt_task_create(&can_receive_thread, "canReceive", 0, 51, 0);
    rt_task_create(&rt_control_thread, "rtController",0, 52,0);
    

    rt_task_start(&bt_record_thread, &biotacRecordThread, NULL);
    rt_task_start(&can_receive_thread, &canReceiveThread, NULL);
    rt_task_start(&rt_control_thread, &rtControlThread, NULL);

	// Create execution manager this sets the time per operate(period)
	systems::RealTimeExecutionManager mem(0.005, 50);
    
    // Instantiate Systems
    BCProductManager peSys;
	//systems::PrintToStream<double> printSys(&mem, "Result: ");
    systems::Ramp time(&mem, 1.0);
    
    //Build TupleGrouper
    systems::TupleGrouper<double, int, math::Vector< NUM_JOINT_VAR_PASSED >::type, math::Vector<BT_NUM_VAR_RECORD>::type> tg;
    
	connect(time.output, tg.getInput<0>());
    connect(peSys.output, tg.getInput<1>());
    connect(peSys.jointsOutput, tg.getInput<2>());
    connect(btEoSys.output, tg.getInput<3>());
    
    connect(eoSys.output, peSys.input);
    
    typedef boost::tuple<double, int, math::Vector< NUM_JOINT_VAR_PASSED >::type, math::Vector<BT_NUM_VAR_RECORD>::type> tuple_type;
    
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(&mem,new log::RealTimeWriter<tuple_type>(tmpFile,PERIOD_MULTIPLIER * mem.getPeriod()), PERIOD_MULTIPLIER);
    
    
	// Make connections between Systems
	
	
    //systems::connect(peSys.output, printSys.input);
    
    // Push data into peSys' input and run an execution cycle,
	// causing peSys::operate() to be called

    std::cout << "is running - ";
    std::cout << mem.isRunning() << std::endl;
    if(!biotacOff)
    {
        while(!biotacInit)
        {
            usleep(100000);
        }
    }
    printf("Press [Enter] to continue ... sets shouldStart = TRUE");
	fflush(stdout);
	getchar();
    
    
    if (!DEBUG){
        bairClaw.digit[0].FEmotor.SetCurrentLimit(MAXCURRENTALLOWED);
        bairClaw.digit[0].FEmotor.enable();
        bairClaw.digit[0].FEmotor.ActivateCurrentMode(5000, MAXCURRENTALLOWED, 25000);
        bairClaw.digit[0].ADABmotor.SetCurrentLimit(MAXCURRENTALLOWED);
        bairClaw.digit[0].ADABmotor.ActivateCurrentMode(5000, MAXCURRENTALLOWED, 25000);
        bairClaw.digit[0].ADABmotor.enable();
    }else{

    }
 
    // Start loadCellRecordThread
    boost::thread t(loadcellRecordThread), BCMatlabVis(bairClawMatlabVisualizerSender), d(displayThread);
 
    
    shouldStart = true;
    int j=0;
    
    
    eoSys.setValue(jointInput);

    
    //bairClaw.digit[0].setStaticFriction();
    printf(" Static setFriction complete MCP-F - %d MCP-E - %d \n Press [Enter] to start logging", bairClaw.digit[0].FEmotor.staticFrictionF, bairClaw.digit[0].FEmotor.staticFrictionE);
	fflush(stdout);
	getchar();
    
   


    
    mem.startManaging(peSys);
    mem.start();
    
    time.start();
    
    // setTendonOffsets -------------------------------------------------
    btsleep(0.5);
    bairClaw.digit[0].setTendonForceOffset();
    printf("Press [Enter] to START display");
    fflush(stdout);
    getchar();
    
    displayOn = true; // has display thread clear and print to screen
    connect(tg.output, logger.input);

	printf("Logging started.\n");
    
    if(!DEBUG)
    {
        // Follow data2Track.bin
    }
    else
    {
    
    }
    
    ready = true;
    cv.notify_all();

    
    printf("Press [Enter] to STOP logging");
	fflush(stdout);
	getchar();

   
    bairClaw.digit[0].FEmotor.resetAll();
    going = false;
    
    d.join(); //Stop display thread
    BCMatlabVis.join();
    t.interrupt(); //Use interrupt instead of join() becasue recvfrom is blocking and if we lose network won't exit.
    std::cout << " About to remove tasks" << std::endl;

    delete playbackData;
    rt_task_delete(&bt_record_thread);
    std::cout << " bt_record_returned" << std::endl;
    rt_task_delete(&can_receive_thread);
    std::cout << " can_receive_returned" << std::endl;
    rt_task_delete(&rt_control_thread);
    std::cout << " rt_control_returned" << std::endl;
 

    
    
    std::cout << " t joined" << std::endl;
    mem.stop();
    std::cout << "is running - ";
    std::cout << mem.isRunning() << std::endl;

    
    
    
    logger.closeLog();
    
    
	printf("Logging stopped.\n");
    
	log::Reader<tuple_type> lr(tmpFile);
    //Add file time stamp
    std::string fileName = "output_";
    time_t nowR;
    char timeNow[50];
    timeNow[0] ='\0';
    nowR = std::time(NULL);
    if(nowR != -1)
    {
        strftime(timeNow, 50, "%m-%d-%Y_%H-%M", localtime(&nowR));
        fileName.append(std::string(timeNow));
        fileName.append(".csv");
    }
    
	lr.exportCSV(fileName.c_str());
	printf("Output written to %s.\n", fileName.c_str());
	std::remove(tmpFile);
    
    std::cout << "About to print Period - ";
    
    std::cout << mem.getPeriod() << " x - " <<  x << std::endl;

	
    
	return 0;
}
