/*=========================================================================
 | (c) 2013  Arizona State University Biomechatronics Lab
 |--------------------------------------------------------------------------
 | Project : Bair Claw Main/Interface
 | File    : BairClawMain.c
 | Authors : Randy Hellman (RBHellman@gmail.com, RHellman@asu.edu)
 Veronica Santos (Veronica.Santos@asu.edu)
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
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>
#include <unistd.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
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
//EPOS REQUIRED
#include "EPOSInterface.h"
#include "BairClawInterface.h"



using namespace barrett;
using systems::connect;

const int NUM_JOINTS = 4;
bool going = true;



class BCProductManager : public systems::System {
    // IO
    // Marked as "public" because Inputs and Output are (except in special cases)
    // part of a System's public interface.
public:
    Input<math::Vector<NUM_JOINTS>::type> input;
    Output<double> output;
    Output<math::Vector<10>::type> jointsOutput;
    
    int countCheck;
    
    // Marked as "protected" because this object lets us change the value of an
    // Output, which should only be allowed from within this System.
protected:
    Output<double>  ::Value* outputValue;
    Output<math::Vector<10>::type>::Value* jointsOutputValue;
    math::Vector<10>::type joints;
    
    BCHand BairClaw;
public:
	// Every System has a human readable name. It's good practice to provide an
	// appropriate default. Notice that outputValue is associated with output
	// via output's constructor.
	explicit BCProductManager(const std::string& sysName = "BairClawProductManager") : systems::System(sysName), input(this), output(this, &outputValue),jointsOutput(this, &jointsOutputValue), countCheck(0) {}
    
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
    
    math::Vector<NUM_JOINTS>::type jointActual;
	// Implement System::operate(). The operate() function must be declared with
	// the "protected" access specifier.
	virtual void operate() {
		jointActual = input.getValue();  // Pull data from the input
        
        static double count = 0.123456789;
        count++;
        countCheck ++;
        for( int j(0); j< 4; j++)
        {
            joints[j] = jointActual[j];
        }
        for( int j(4); j<10; j++)
        {
            joints[j] = count+j;
        }
        outputValue->setData(&count);
        jointsOutputValue->setData(&joints);
	}
    
    
};

 #pragma mark - canThread
/*
void canThread() {
 tuple_type t;
 
 int id = fts->getPuck()->getId();
 const bus::CommunicationsBus& bus = fts->getPuck()->getBus();
 
 rt_task_shadow(new RT_TASK, NULL, 10, 0);
 
 
 // collect data
 rt_task_set_mode(0, T_PRIMARY | T_WARNSW, NULL);
 rt_task_set_periodic(NULL, TM_NOW, secondsToRTIME(T_s));
 
 RTIME t_0 = rt_timer_read();
 RTIME t_1 = t_0;
 
 while (going) {
 rt_task_wait_period(NULL);
 
 t_0 = rt_timer_read();
 boost::get<0>(t) = ((double) t_0 - t_1) * 1e-9;
 t_1 = t_0;
 
 
 // strain gauges
 boost::get<1>(t)[0] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG1, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[1] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG2, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[2] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG3, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[3] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG4, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[4] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG5, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[5] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::SG6, Puck::PT_ForceTorque, 0), true);
 
 // temp sensors
 boost::get<1>(t)[6] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T1, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[7] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T2, Puck::PT_ForceTorque, 0), true);
 boost::get<1>(t)[8] = fts->getPuck()->getProperty(bus, id, Puck::getPropertyId(Puck::T3, Puck::PT_ForceTorque, 0), true);
 
 
 if (fileMode) {
 lw->putRecord(t);
 } else {
 if (numSamples == 0) {
 sum.setConstant(0.0);v
 }
 if (numSamples < windowSize) {
 sum += boost::get<1>(t);
 ++numSamples;
 }
 }
 }
 
 rt_task_set_mode(T_WARNSW, 0, NULL);
 }*/

#pragma mark - mainEntryPoint

int main() {
    //make temp logging file info
    char tmpFile[] = "/tmp/btXXXXXX";
	if (mkstemp(tmpFile) == -1)
    {
		printf("ERROR: Couldn't create temporary file!\n");
		return 1;
	}
    
    math::Vector<NUM_JOINTS>::type jointInput;
    for(int k(0); k<NUM_JOINTS; k++)
    {
        jointInput[k] = k;
    }
    
    
	// Create execution manager
	systems::RealTimeExecutionManager mem(0.005, 50);
    
    // Instantiate Systems
	systems::ExposedOutput<math::Vector<NUM_JOINTS>::type> eoSys;
    BCProductManager peSys;
	//systems::PrintToStream<double> printSys(&mem, "Result: ");
    
    systems::Ramp time(&mem, 1.0);
    
    //Build TupleGrouper
    systems::TupleGrouper<double, double, math::Vector<10>::type> tg;
    
	connect(time.output, tg.getInput<0>());
    connect(peSys.output, tg.getInput<1>());
    connect(peSys.jointsOutput, tg.getInput<2>());
    
    
    typedef boost::tuple<double, double, math::Vector<10>::type> tuple_type;
    
	const size_t PERIOD_MULTIPLIER = 1;
	systems::PeriodicDataLogger<tuple_type> logger(&mem,new log::RealTimeWriter<tuple_type>(tmpFile,PERIOD_MULTIPLIER * mem.getPeriod()), PERIOD_MULTIPLIER);
    
    
	// Make connections between Systems
	systems::connect(eoSys.output, peSys.input);
	//systems::connect(peSys.output, printSys.input);
    
    
    // Push data into peSys' input and run an execution cycle,
	// causing peSys::operate() to be called
	
    eoSys.setValue(jointInput);
    
    mem.startManaging(peSys);
    mem.start();
    
    std::cout << "is running - ";
    std::cout << mem.isRunning() << std::endl;
    
    time.start();
    
    //-------------------------------------------------
    
    
    
    connect(tg.output, logger.input);
	printf("Logging started.\n");
    
    
    
    for(int i(0); i < 10; i++)
    {
        btsleep(0.5);
        std::cout << peSys.getCount() << " time - " << std::endl;
        btsleep(0.5);
        std::cout << peSys.getCount() << std::endl;
        std::cout << "is running - ";
        std::cout << mem.isRunning() << std::endl;
    }
    mem.stop();
    std::cout << "is running - ";
    std::cout << mem.isRunning() << std::endl;
    /*
     eoSys.setValue(-0.5);
     mem.runExecutionCycle();
     eoSys.setValue(0.0);
     mem.runExecutionCycle();
     eoSys.setValue(0.5);
     mem.runExecutionCycle();
     eoSys.setValue(1.0);
     mem.runExecutionCycle();
     */
    logger.closeLog();
	printf("Logging stopped.\n");
    
	log::Reader<tuple_type> lr(tmpFile);
	lr.exportCSV("testoutput.txt");
	printf("Output written to %s.\n", "testoutput.txt");
	std::remove(tmpFile);
    
    std::cout << "About to print Period - ";
    std::cout << mem.getPeriod() << std::endl;
	
    
	return 0;
}
