/*
 * two_wams.cpp
 *
 *  Created on: Feb 15, 2011
 *      Author: dc
 */

#include <iostream>
#include <vector>
#include <string>
#include <stdio.h>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/os.h>
#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;
bool  waitForGravitComp           = true;
bool  shouldStartTeaching         = false;
bool  shouldStopTeaching          = false;
bool  shouldReplayTrajectory      = false;
bool  shouldSendHome              = false;
bool  shouldRerunTrajectory       = true;
bool  shouldSendToTrajectoryStart = false;
bool  currentlyReruningTrajectory = false;
bool  shouldInitHands             = false;


boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm0;
	ProductManager pm1("/etc/barrett/bus1/default.conf");

    std::cout << "Starting the WAM on Bus 0...\n" << std::endl;
	boost::thread wt0 = startWam(pm0, wamThread0<4>, wamThread0<7>);
    std::cout << "Starting the WAM on Bus 1...\n" << std::endl;
	boost::thread wt1 = startWam(pm1, wamThread1<4>, wamThread1<7>);
    
    std::cout << "Press [Enter] to gravity comp both WAMs" << std::endl;
    waitForEnter();
    waitForGravitComp = false;
    
    std::cout << "Press [Enter] to init hands" << std::endl;
    waitForEnter();
    shouldInitHands = true;
    
    std::cout << "Press [Enter] to start teaching both WAMs" << std::endl;
    waitForEnter();
    shouldStartTeaching = true;
    
    std::cout << "Press [Enter] to stop teaching both WAMs" << std::endl;
    waitForEnter();
    shouldStopTeaching = true;
    
    std::cout << "Press [Enter] to play recorded trajectory" << std::endl;
    waitForEnter();
    shouldReplayTrajectory = true;
    currentlyReruningTrajectory = true;
    
    while(currentlyReruningTrajectory)
    {
        btsleep(.1);
    }
    std::cout << "Press [Enter] to send WAM to start of trajectory" << std::endl;
    waitForEnter();
    shouldSendToTrajectoryStart = true;
    bool going = true;
    std::string line;
    while(going)
    {
        std::cout << "Would to like to rerun trajectoy ('y')" << std::endl;
        printf(">>> ");
        char c = getchar();
        if (c == 'y')
        {
            std::cout << "Press [Enter] to play recorded trajectory" << std::endl;
            waitForEnter();
            shouldReplayTrajectory = true;
            currentlyReruningTrajectory = true;
            std::cout << "Press [Enter] to send WAM to start of trajectory" << std::endl;
            waitForEnter();
            shouldSendToTrajectoryStart = true;
            
            
        }
        else{
            going = false;
            shouldReplayTrajectory = true;
            shouldRerunTrajectory  = false;
        }
    }
    
    std::cout << "Press [Enter] to send WAM home\n" << std::endl;
    waitForEnter();
    shouldSendHome = true;
    
    
    
	wt0.join();
	wt1.join();

	return 0;
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
    Hand& hand = *pm.getHand();
	while(waitForGravitComp)
    {
        btsleep(0.1);
    }
    wam.gravityCompensate();
    
    if ( !pm.foundHand() ) {
        printf("ERROR: No Hand found on bus!\n");
    }
    else
    {
        std::cout << "FOUND Hand on bus - " << pm.getWamDefaultConfigPath() << "  -" << std::endl;
    }
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
    
    while(!shouldInitHands)
    {
        btsleep(0.1);
    }
    wam.moveTo(wam.getJointPositions());
    hand.initialize();
    hand.waitUntilDoneMoving();
    hand.close();
    btsleep(1);
    wam.idle();
    
    
    typedef boost::tuple<double, jp_type> jp_sample_type;
    
    char tmpFile[] = "/tmp/btXXXXXX";
    if (mkstemp(tmpFile) == -1) {
        printf("ERROR: Couldn't create temporary file!\n");
    }
    
    const double T_s = pm.getExecutionManager()->getPeriod();
    
    
    systems::Ramp time(pm.getExecutionManager());
    
    systems::TupleGrouper<double, jp_type> jpLogTg;
    
    // Record at 1/10th of the loop rate
    systems::PeriodicDataLogger<jp_sample_type> jpLogger(pm.getExecutionManager(),
                                                         new barrett::log::RealTimeWriter<jp_sample_type>(tmpFile, 10*T_s), 10);
    
    
    //printf("Press [Enter] to start teaching.\n");
    //waitForEnter();
    while(!shouldStartTeaching)
    {
        btsleep(0.01);
    }
    {
        // Make sure the Systems are connected on the same execution cycle
        // that the time is started. Otherwise we might record a bunch of
        // samples all having t=0; this is bad because the Spline requires time
        // to be monotonic.
        BARRETT_SCOPED_LOCK(pm.getExecutionManager()->getMutex());
        
        connect(time.output, jpLogTg.template getInput<0>());
        connect(wam.jpOutput, jpLogTg.template getInput<1>());
        connect(jpLogTg.output, jpLogger.input);
        time.start();
    }
    
    while(!shouldStopTeaching)
    {
        btsleep(0.01);
    }
    //printf("Press [Enter] to stop teaching.\n");
    //waitForEnter();
    jpLogger.closeLog();
    disconnect(jpLogger.input);
    
    
    // Build spline between recorded points
    log::Reader<jp_sample_type> lr(tmpFile);
    std::vector<jp_sample_type> vec;
    for (size_t i = 0; i < lr.numRecords(); ++i) {
        vec.push_back(lr.getRecord());
    }
    math::Spline<jp_type> spline(vec);
    

    //printf("Press [Enter] to play back the recorded trajectory.\n");
    //waitForEnter();
    
    // First, move to the starting position
    wam.moveTo(spline.eval(spline.initialS()));
    while(shouldRerunTrajectory)
    {
        while(!shouldReplayTrajectory)
        {
            btsleep(0.01);
        }

        // Then play back the recorded motion
        time.stop();
        time.setOutput(spline.initialS());
        
        systems::Callback<double, jp_type> trajectory(boost::ref(spline));
        connect(time.output, trajectory.input);
        wam.trackReferenceSignal(trajectory.output);
        
        time.start();
        
        while (trajectory.input.getValue() < spline.finalS()) {
            usleep(100000);
        }
        shouldReplayTrajectory = false;
        currentlyReruningTrajectory = false;
        while(!shouldSendToTrajectoryStart)
        {
            btsleep(0.01);
        }
        
        wam.moveTo(spline.eval(spline.initialS()));
        while(!shouldReplayTrajectory)
        {
            btsleep(0.01);
        }
        shouldSendToTrajectoryStart = false;
        
    }
    hand.trapezoidalMove(open, Hand::GRASP);
    hand.waitUntilDoneMoving();
    //printf("Press [Enter] to send WAM home\n");
    //waitForEnter();
    while(!shouldSendHome)
    {
        btsleep(0.1);
    }
    
    
    wam.moveHome();

    
    std::remove(tmpFile);
    
    
    //jp_type jp(0.0);
	while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		/*wam0.moveTo(jp);
		sleep(1);
		wam0.moveHome();*/
		btsleep(1);
	}
}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}
























