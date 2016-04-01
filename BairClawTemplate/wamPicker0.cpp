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
#include <barrett/products/product_manager.h>


using namespace barrett;
using detail::waitForEnter;
int wamPicker;

boost::thread startWam(ProductManager& pm,
		boost::function<void (ProductManager&, systems::Wam<4>&)> wt4,
		boost::function<void (ProductManager&, systems::Wam<7>&)> wt7);
template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0);



int main(int argc, char** argv) {
	// Give us pretty stack-traces when things die
	installExceptionHandler();
    wamPicker = atoi(argv[1]);
    
    ProductManager *pm;
    

	printf("Starting the WAM on Bus %d...\n",wamPicker);
    if(wamPicker == 1)
    {
        ProductManager pm1("/etc/barrett/bus1/default.conf");
        pm = &pm1;
    }
    else{
        ProductManager pm0;
        pm = &pm0;
    }
    
    boost::thread wt0 = startWam(*pm, wamThread0<4>, wamThread0<7>);
	wt0.join();


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


template <size_t DOF> void wamThread0(ProductManager& pm0, systems::Wam<DOF>& wam0) {
	BARRETT_UNITS_TEMPLATE_TYPEDEFS(DOF);

	wam0.gravityCompensate();

	jp_type jp(0.0);
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

	while (pm0.getSafetyModule()->getMode() == SafetyModule::ACTIVE) {
		wam0.moveTo(jp);
        printf("Press [Enter] to return home.");
        waitForEnter();
		wam0.moveHome();
		sleep(1);
	
    
    
    
    }
    
    
    
    
    
}

template <size_t DOF> void wamThread1(ProductManager& pm1, systems::Wam<DOF>& wam1) {
	wamThread0(pm1, wam1);
}

