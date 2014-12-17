/*
 * two_wams.cpp
 *
 *  Created on: Feb 15, 2011
 *      Author: dc
 */

#include <iostream>
#include <fstream>
#include <string>

#include <boost/thread.hpp>
#include <boost/function.hpp>

#include <barrett/exception.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/products/product_manager.h>


using namespace barrett;
using namespace std;
using detail::waitForEnter;
int wamPicker;


boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);
template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1);


int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();

    ProductManager *pm0, *pm1;

    
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
    }
    
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

void printMenu() {
    printf("\nCommands:\n");
    printf("  l  Log joint position for trajectory maker\n");
    printf("  H  Move to the home position\n");
    printf("  i  Idle (release position/orientation constraints)\n");
    printf("  h  Hold Position\n");
    printf("  f  initialize and open hand\n");
    printf("  q  Quit\n");
}


template <size_t DOF> void wamThread0(ProductManager& pm, systems::Wam<DOF>& wam) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

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
    /////////////////////////////
    
    wam.gravityCompensate();
    wam.moveHome();
	jp_type jp(0.0), jplogger(0.0), jphold(0.0);
    
    /*
    if(wamPicker == 1)
    {
        jp[0] = M_PI_2;
    }
    else
    {
        jp[0] = -M_PI_2;
    }
    jp[1] = -2.0;
    jp[3] =  2.2;
     */
    //Only run this section of code if not both wam are running
    printf("Press [Enter] to start wam thread");
    waitForEnter();
    
    if(wamPicker != 3)
    {
        printMenu();
        ofstream myfile("WAMPositionLogForQuickSetup.txt");

        string line;
        bool going = true;
        while (going) {
            printf(">>> ");
            std::getline(std::cin, line);
            
            switch (line[0]) {
                case 'l':
                    /*
                    char buf[100];
                    sprintf(buf,"JointPosFileWAM_%d",wamPicker);
                    printf("Logging in file: %s",buf);
                    ofstream myfile("testLog.txt");
                    */
                    if (myfile.is_open())
                    {
                        printf("    Joint Positions (rad): ");
                        jplogger = math::saturate(wam.getJointPositions(), 9.999);
                        printf("[%6.3f", jp[0]);
                        for (size_t i = 1; i < DOF; ++i) {
                            printf(", %6.3f", jplogger[i]);
                        }
                        printf("]");
                        
                        myfile << "[ ";
                        for (size_t i = 0; i < DOF; ++i) {
                            myfile << jplogger[i] << " ";
                            
                        }
                        myfile << "]" << "\n";
                    }
                    else cout << "Unable to open file";
                    break;

                case 'H':
                    std::cout << "Moving to home position: " << wam.getHomePosition() << std::endl;
                    wam.moveHome();
                    break;
                 
                case 'h':
                    std::cout << "WAM Hold Position: " << wam.getHomePosition() << std::endl;
                    jphold = math::saturate(wam.getJointPositions(), 9.999);
                    wam.moveTo(jphold);
                    break;
                    
                case 'i':
                    printf("WAM idled.\n");
                    wam.idle();
                    break;
                case 'f':

                    hand.initialize();
                    hand.trapezoidalMove(open, Hand::SPREAD);
                    hand.trapezoidalMove(open, Hand::GRASP);
                    printf("Hand done initializing and now open");
                    break;
                    
                case 'q':
                case 'x':
                    hand.trapezoidalMove(open, Hand::GRASP);
                    hand.trapezoidalMove(close, Hand::SPREAD);
                    hand.trapezoidalMove(partial, Hand::GRASP);
                    hand.idle();
                    
                    printf("Quitting.\n");
                    going = false;
                    break;
                    
                default:
                    if (line.size() != 0) {
                        printf("Unrecognized option.\n");
                        printMenu();
                    }
                    break;
            }
        }
        myfile.close();

    }
    else
    {
    wam.moveTo(jp);
    }
    printf("Press [Enter] to return home.");
    waitForEnter();
    wam.moveHome();
    pm.getSafetyModule()->waitForMode(SafetyModule::IDLE);


}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}

