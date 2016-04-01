/*
 * can_terminal.cpp
 *
 *  Created on: Aug 19, 2010
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <barrett/exception.h>
#include <boost/tuple/tuple.hpp>
#include <boost/thread.hpp>
#include <boost/function.hpp>
#include <unistd.h>
#include <barrett/os.h>  // For btsleep()
#include <barrett/bus/can_socket.h>
#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>


//Data logging
#define BARRETT_SMF_VALIDATE_ARGS

//BIOTAC REQUIRED
//#include "cheetah.h"
//#include "biotac.h"

using namespace barrett;
using namespace std;
using detail::waitForEnter;
using detail::waitForEnter;
using systems::connect;
using systems::disconnect;
using systems::reconnect;

int MovementState = 0;
bool BTrecord = false;
char BTfilename[100];
char BHfilename[100];
char WAMfilename[120];
int BTn=0;
int trialNum = 0;

bool going = false;

std::vector<std::string> trajFileNames;





void loadTrajFileNames() 
{
	std::string line;
	int trial(0);
	std::ifstream myfile("trajFile.txt");

	if (myfile.is_open())
	{
	    while ( myfile.good() )
	    {
	    	std::getline(myfile,line);

	    	trajFileNames.push_back(line);

	    }
	    myfile.close();
  }
  else std::cout << "Unable to open file"; 
}





boost::thread startWam(ProductManager& pm,
                       boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
                       boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);

int main(int argc, char** argv) {
    // Give us pretty stack-traces when things die
    installExceptionHandler();
	
    ProductManager *pm0, *pm1;
    //Only
    ProductManager pm1l("/etc/barrett/bus1/default.conf");
    pm1 = &pm1l;
    printf("Starting the WAM on Bus 1...\n");
    boost::thread wt1l = startWam(*pm1, wamThread1<4>, wamThread1<7>);
    wt1l.join();
    
    /*///////////////////////////////////////////
    if(argc >= 2)
    {
        wamPicker = atoi(argv[1]);
    }
    if(wamPicker == 0)
    {//Start only wam0
        ProductManager pm0l;
        pm0 = &pm0l;
        printf("Starting the WAM on Bus 0...\n");
        //boost::thread wt0l = startWam(*pm0, wam0_state, wamThread0<4>, wamThread0<7>);
        boost::thread wt0l = startWam(*pm0, wamThread0<4>, wamThread0<7>);
        wt0l.join();
        
    }
    else if(wamPicker == 1)
    {//Start only wam1
        ProductManager pm1l("/etc/barrett/bus1/default.conf");
        pm1 = &pm1l;
        printf("Starting the WAM on Bus 1...\n");
        boost::thread wt1l = startWam(*pm1, wamThread1<4>, wamThread1<7>);
        wt1l.join();
        
    }
    else
    {//Start both wam1 & wam2
        wamPicker = 3;
        ProductManager pm0l;
        pm0 = &pm0l;
        printf("Starting the WAM on Bus 0...\n");
        boost::thread wt0l = startWam(*pm0, wamThread0<4>, wamThread0<7>);
        
        
        ProductManager pm1l("/etc/barrett/bus1/default.conf");
        pm1 = &pm1l;
        printf("Starting the WAM on Bus 1...\n");
        boost::thread wt1l = startWam(*pm1, wamThread1<4>, wamThread1<7>);
        
        wt0l.join();
        wt1l.join();
    }*//////////////////////////////////////////
    
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
	
	bool fileOpen = false;
	char input[255];
	int err, len, SG3;

    ////// HAND//////////////////
    if ( !pm.foundHand() ) {
        printf("ERROR: No Hand found on bus!\n");
    }
    std::cout << "FOUND Hand on bus - " << pm.getWamDefaultConfigPath() << "  -" << std::endl;
    Hand& hand = *pm.getHand();
    typedef Hand::jp_type hjp_t;
    hjp_t open(0.0);
    hjp_t close(2.4);
    double SC = M_PI;
    close[3] = SC;
    hjp_t partial(1.3);
    jp_type jp(0.0), jpStart(0.0), jphold(0.0);
    /////////////////////////////
    
    
    wam.gravityCompensate();
    wam.moveHome();
    hand.initialize();
    hand.trapezoidalMove(open, Hand::SPREAD);
    hand.trapezoidalMove(open, Hand::GRASP);

    
	typedef boost::tuple<double, jp_type> jp_sample_type;
	going = true;

	printf("\npress [Enter] to move to start above turntable.");
	waitForEnter();
    jp[0] =  -0.94727;
    jp[1] =  1.60449;
    jp[2] =  -1.55779;
    jp[3] =  2.08408;
    jp[4] =  0.146203;
    jp[5] =  0.251684;
    jp[6] =  0.113739;
    wam.moveTo(jp);
    
    
//-0.8373    1.6050   -1.6235    1.7704    0.1212    0.5383   -0.1141
//    jpStart[0] =  -1.3312;
//    jpStart[1] =  1.60726;
//    jpStart[2] =  -1.55268;
//    jpStart[3] =  1.96392;
//    jpStart[4] =  0.243618;
//    jpStart[5] =  -0.0242749;
//    jpStart[6] =  0.00565097;
    
    
    jpStart[0] = -1.4296;
    jpStart[1] = 1.5787;
    jpStart[2] = -1.5843;
    jpStart[3] = 1.5970;
    jpStart[4] = 0.1253;
    jpStart[5] = 0.1159;
    jpStart[6] = -0.1185;

    wam.moveTo(jpStart);

	//boost::thread threadBarrettHand(BarrettHandSGThread, &going);
	//boost::thread threadBioTac(BioTacThread, &going);

	systems::Ramp time(pm.getExecutionManager());
//---- MOVE to starting point ---------------------------------------------------------------------
//	log::Reader<jp_sample_type> lr("firstMove");
//	std::vector<jp_sample_type> vec;
//	for (size_t i = 0; i < lr.numRecords(); ++i) {
//		vec.push_back(lr.getRecord());
//	}
//	//INITALIZE system with hold point and number of iterations.
//	printf("Loaded firstmove\n");
//
	int numOfRuns(1);
	printf("\nHow many iteration would you like? - ");
	std::cin >> numOfRuns;
//	printf("\nEnter trial number to be added to filenames - ");
//	std::cin >> trialNum;
	printf("\nWould you like hold points off? (y) any other value is no...");
	char check = 'n';
	std::cin >> check;
	waitForEnter();
	int holdPoint(1);
	
	if( check == 'y' )
	{	
		printf("\nHold points OFF! be careful!");
		holdPoint =0;
	}
//	if(holdPoint)
//	{
//		printf("\nCheck that firstmove vec makes sense numRecords = %4.0ld\n Press [Enter] to continue...",lr.numRecords());
//		waitForEnter();
//	}
//	math::Spline<jp_type> spline(vec);
//	// First, move to the starting position
//	wam.moveTo(spline.eval(spline.initialS()));
//	// Then play back the recorded motion
//	time.stop();
//	time.setOutput(spline.initialS());
//
//	systems::Callback<double, jp_type> trajectory(boost::ref(spline));
//	connect(time.output, trajectory.input);
//	wam.trackReferenceSignal(trajectory.output);
//	time.start();
//	while (trajectory.input.getValue() < spline.finalS()) {
//		usleep(100000);
//	}



// Logger setup --------------------------------------------------------------------		
//	systems::TupleGrouper<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tg;
//	connect(time.output, tg.template getInput<0>());
//	connect(wam.jpOutput, tg.template getInput<1>());
//	connect(wam.jvOutput, tg.template getInput<2>());
//	connect(wam.jtSum.output, tg.template getInput<3>());
//	connect(wam.toolPosition.output, tg.template getInput<4>());
//	connect(wam.toolOrientation.output, tg.template getInput<5>());

//	typedef boost::tuple<double, jp_type, jv_type, jt_type, cp_type, Eigen::Quaterniond> tuple_type;
//	const size_t PERIOD_MULTIPLIER = 5;

    printf("\nEnd of firstmove, press [Enter] to initialize hand and set ball in place.");
    waitForEnter();
    
	loadTrajFileNames();
	printf("trajFileName size: %d\n",trajFileNames.size()-1);
	for(int i(0); i < trajFileNames.size()-1; i++)
	{
		printf("    -%s-\n",trajFileNames[i].c_str());
	}
//Load iter-------------------------------------------------------------------------
	for(int i(0); i < trajFileNames.size()-1; i++)
	{
		log::Reader<jp_sample_type> lr2(trajFileNames[i].c_str());
		std::vector<jp_sample_type> vec2;
		for (size_t i = 0; i < lr2.numRecords(); ++i) {
			vec2.push_back(lr2.getRecord());
		}
		math::Spline<jp_type> spline2(vec2);
		if(holdPoint)
		{
			printf("\nCheck that iter vec makes sense numRecords = %4.0ld\n Press [Enter] to continue...",lr2.numRecords());
			waitForEnter();
		}
        wam.moveTo(spline2.eval(spline2.initialS()));
		systems::Callback<double, jp_type> trajectory2(boost::ref(spline2));
		connect(time.output, trajectory2.input);
	//---------------------------------------------------------------------------------------

		for(int j(0); j<numOfRuns; j++)
		{
			if(holdPoint)
			{
				printf("\nPress [ENTER] to start iteration # %4.0d\n",j+1);
				waitForEnter();
			}

			btsleep(0.5);

			time.stop();
			wam.trackReferenceSignal(trajectory2.output);
			time.setOutput(spline2.initialS());
			//connect(tg.output, logger.input);
			time.start();
			while (trajectory2.input.getValue() < spline2.finalS()) {
				usleep(100000);
			}

			btsleep(0.25);

			wam.moveTo(spline2.eval(spline2.initialS()));

			btsleep(0.25);
			//Close and save log 
			//logger.closeLog();
			//printf("Logging stopped.\n");
			//log::Reader<tuple_type> lrr(tmpFile);
			//lrr.exportCSV(WAMfilename);
			//printf("Output written to %s.\n", WAMfilename);
			//std::remove(tmpFile);
		}
	}

 
    if(holdPoint)
    {
        printf("\nDone with iterations. Press [ENTER] to return WAM to home.");
        waitForEnter();
    }
    wam.moveTo(jp);
    wam.moveHome();
    hand.trapezoidalMove(open, Hand::GRASP);
    hand.trapezoidalMove(close, Hand::SPREAD);
    hand.trapezoidalMove(partial, Hand::GRASP);
    hand.idle();


    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);
	going = false;

//	threadBarrettHand.join();
//  threadBioTac.join();


}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
    wamThread0(pm1, wam1);
}

