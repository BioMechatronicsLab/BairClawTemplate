
#include "BairClawInterface.h"

namespace barrett {

//############################################################################################################
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////         BAIRCLAW Implementation          /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////
//############################################################################################################

void BCInitThread( BCDigit* digit, const bool* initgoing) {
    
	int thrdCnt = 0;
	char input;
	system("clear");
	printf("\nWould you like to run initialization? [y or n (use default ranges) ]: \n");
	input = getchar();
    
	if(input == 'y')
	{
		digit->AdAbmin = digit->AdAb;
		digit->FEmin   = digit->FE;
		digit->DIPmin  = digit->DIP;
		digit->PIPmin  = digit->PIP;
		digit->AdAbmax = digit->AdAb;
		digit->FEmax   = digit->FE;
		digit->DIPmax  = digit->DIP;
		digit->PIPmax  = digit->PIP;
        
		while (*initgoing) {
			thrdCnt++;
			// MIN
			if( (digit->AdAb) < (digit->AdAbmin))   digit->AdAbmin = digit->AdAb;
			if( (digit->FE)   < (digit->FEmin  ))   digit->FEmin = digit->FE;
			if( (digit->DIP)  < (digit->DIPmin ))   digit->DIPmin = digit->DIP;
			if( (digit->PIP)  < (digit->PIPmin ))   digit->PIPmin = digit->PIP;
			// MAX
			if( (digit->AdAb) > (digit->AdAbmax))   digit->AdAbmax = digit->AdAb;
			if( (digit->FE)   > (digit->FEmax  ))   digit->FEmax = digit->FE;
			if( (digit->DIP)  > (digit->DIPmax ))   digit->DIPmax = digit->DIP;
			if( (digit->PIP)  > (digit->PIPmax ))   digit->PIPmax = digit->PIP;
			//
            
			if(thrdCnt % 5 == 0)
			{
				system("clear");
				printf("README: Cycle all joints until you have reached the \n        desired min and max for each joint.\n\n");
				printf("        AdAb     FE    PIP     DIP\n");
				printf("Current %d     %d     %d      %d \n", digit->AdAb   , digit->FE   , digit->PIP   , digit->DIP   );
				printf(" min    %d     %d     %d      %d \n", digit->AdAbmin, digit->FEmin, digit->PIPmin, digit->DIPmin);
				printf(" max    %d     %d     %d      %d \n", digit->AdAbmax, digit->FEmax, digit->PIPmax, digit->DIPmax);
			}
			usleep(5000);
            
		}
		printf("exiting initThread\n");
	}
	else
	{
		printf("press [Enter] to continue to joint angle visualization.\n");
	}
}
void BCDigit::init(){
	bool initgoing = 1;
	printf("About to start init thread ");
	boost::thread initthread(BCInitThread, &(*this), &initgoing);
	detail::waitForEnter();
	isInit = 1;
	initgoing = false;
	initthread.join();
}

void BairClawVisThread( BCDigit* digit, const bool* visgoing) {
    /*
     int thrdCnt = 0;
     
     digit->AdAbRange = (double)digit->AdAbmax - (double)digit->AdAbmin;
     digit->FERange   = (double)digit->FEmax   - (double)digit->FEmin;
     digit->PIPRange  = (double)digit->PIPmax  - (double)digit->PIPmin;
     digit->DIPRange  = (double)digit->DIPmax  - (double)digit->DIPmin;
     
     
     while (*visgoing) {
     thrdCnt++;
     digit->AdAbPercent = 100 - ( ((double)digit->AdAbRange - ((double)digit->AdAb - (double)digit->AdAbmin)) / (double)digit->AdAbRange ) *100;
     digit->FEPercent =    ( ((double)digit->FERange   - ((double)digit->FE   - (double)digit->FEmin))   / (double)digit->FERange )   *100;
     digit->PIPPercent =  100 - ( ((double)digit->PIPRange  - ((double)digit->PIP  - (double)digit->PIPmin))  / (double)digit->PIPRange )  *100;
     digit->DIPPercent =   ( ((double)digit->DIPRange  - ((double)digit->DIP  - (double)digit->DIPmin))  / (double)digit->DIPRange )  *100;
     
     
     system("clear");
     printf("README: Joint Visualization. Press [Enter] if initialization is acceptable.\n\n");
     printf("AdAb_per = %4.2f, AdAbReadOut = %d\n", digit->AdAbPercent, digit->AdAb);
     printf("FE_per = %4.2f, FEReadOut = %d\n", digit->FEPercent, digit->FE);
     printf("PIP_per = %4.2f, PIPReadOut = %d\n", digit->PIPPercent, digit->PIP);
     printf("DIP_per = %4.2f, DIPReadOut = %d\n\nAd/Ab\n", digit->DIPPercent, digit->DIP);
     for(int i=0;i<digit->AdAbPercent;i++) printf("|");
     printf("\n");
     for(int i=0;i<digit->AdAbPercent;i++) printf("|");
     printf("\nF/E\n");
     for(int i=0;i<digit->FEPercent;i++) printf("|");
     printf("\n");
     for(int i=0;i<digit->FEPercent;i++) printf("|");
     printf("\nPIP\n");
     for(int i=0;i<digit->PIPPercent;i++) printf("|");
     printf("\n");
     for(int i=0;i<digit->PIPPercent;i++) printf("|");
     printf("\nDIP\n");
     for(int i=0;i<digit->DIPPercent;i++) printf("|");
     printf("\n");
     for(int i=0;i<digit->DIPPercent;i++) printf("|");
     printf("\n");
     
     usleep(10000);
     }
     printf("exiting initThread\n");
     */
}

void BCDigit::calcPercentage(){
    /*
     AdAbRange = (double)AdAbmax - (double)AdAbmin;
     FERange   = (double)FEmax   - (double)FEmin;
     PIPRange  = (double)PIPmax  - (double)PIPmin;
     DIPRange  = (double)DIPmax  - (double)DIPmin;
     
     AdAbPercent = 100 - ( ((double)AdAbRange - ((double)AdAb - (double)AdAbmin)) / (double)AdAbRange ) *100;
     FEPercent =    ( ((double)FERange   - ((double)FE   - (double)FEmin))   / (double)FERange )   *100;
     PIPPercent =  100 - ( ((double)PIPRange  - ((double)PIP  - (double)PIPmin))  / (double)PIPRange )  *100;
     DIPPercent =   ( ((double)DIPRange  - ((double)DIP  - (double)DIPmin))  / (double)DIPRange )  *100;
     */
}

void BCDigit::vis(){
	/*
     if(isInit)
     {
     bool visgoing = 1;
     printf("About to start visualization thread ");
     boost::thread visthread(BairClawVisThread, &(*this), &visgoing);
     detail::waitForEnter();
     visgoing = false;
     visthread.join();
     }
     else
     {
     printf("Need to initialize digit prior to visualization\n");
     }
     */
}
void BCDigit::setStaticFriction(){
	/*
     system("clear");
     
     printf("Starting static friciton calibration on FE node = %d\n",motorFE.nodeSet);
     int initFE = FE;
     int staticCurrent = 0;
     
     while(abs(initFE - FE) < 10 )
     {
     staticCurrent = staticCurrent + 1;
     motor.SetCurrent(staticCurrent);
     if(staticCurrent % 5 == 0 )
     printf("initFe = %d, FE = %d, Current = %d\n", initFE, FE, staticCurrent);
     usleep(20000);
     }
     if(FE < initFE)
     normalRotation = -1;
     motor.SetCurrent(0);
     printf("Static current = %d\n",staticCurrent);
     usleep(500000);
     staticFrictionF = staticCurrent;
     staticCurrent = 0;
     initFE = FE;
     while(abs(initFE - FE) < 10 )
     {
     staticCurrent = staticCurrent - 1;
     motorFE.SetCurrent(staticCurrent);
     if(staticCurrent % 5 == 0 )
     printf("initFe = %d, FE = %d, Current = %d\n", initFE, FE, staticCurrent);
     usleep(20000);
     }
     staticFrictionE = staticCurrent;
     motor.SetCurrent(0);
     printf("Static current = %d\n",staticCurrent);
     
     //printf("Starting static friciton calibration on AdAb node = %d\n",motorAdAb.nodeSet);
     //printf("Starting static friciton calibration on PIPDIP node = %d\n",motorPIPDIP.nodeSet);
     */
}

void BCDigit::backDrive(){
	/*
     int preFE;
     int count = 0 ;
     int flag;
     int flip;
     int hold=0;
     printf("In backDrive\n");
     while(1){
     
     if(FE > preFE){
     motorFE.SetCurrent(staticFrictionF+10);
     preFE = FE;
     flag = 1;
     hold = 0;
     
     }else if( FE < preFE){
     motorFE.SetCurrent(staticFrictionE-10);
     preFE = FE;
     flag = 2;
     hold = 0;
     }else{
     if(hold++ > 50){
     if(flip++ % 2)
     motorFE.SetCurrent(staticFrictionF);
     else
     motorFE.SetCurrent(staticFrictionE);
     flag = 3;
     }else{
     flag = 4;
     }
     
     }
     count++;
     if( count % 10 == 0){
     printf("FE = %d, preFE = %d, flag = %d, count = %d\n", FE, preFE, flag, count);
     }
     }*/
    
}

} //From namespace barrett{}
