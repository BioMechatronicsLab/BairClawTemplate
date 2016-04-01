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


using namespace barrett;
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

boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

	ProductManager pm0;
	//ProductManager pm1("/etc/barrett/bus1/default.conf");

	printf("Starting the WAM on Bus 0...\n");
	boost::thread wt0 = startWam(pm0, wamThread0<4>, wamThread0<7>);
	printf("Starting the WAM on Bus 1...\n");
	//boost::thread wt1 = startWam(pm1, wamThread1<4>, wamThread1<7>);
    std::cout << "Press [Enter] to init Hand" << std::endl;
    waitForEnter();
    shouldInitHand = true;

    bool going = true;
    std::string line;
    while(going)
    {
        printf("\nGravity comp - 'g'\nHold position 'h' \n");
        printf(">>> ");
        c = getchar();
        
        switch(c)
        {
            case('g'):
                WAMg = true;
                break;
            case('h'):
                WAMh = true;
                break;
            case('s'):
                WAMs = true;
                break;
            case('q'):
            case('Q'):
                going = false;
                shouldKeepRunning = false;
                break;
            default:
                break;
        }
        shouldIterate = true;

    }
    waitForEnter();
    std::cout << "Press [Enter] send WAM home" << std::endl;
    waitForEnter();
    shouldSendWAMHome = true;

    wt0.join();
	//wt1.join();

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

    while (pm.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {

		sleep(1);
	}

}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}




















