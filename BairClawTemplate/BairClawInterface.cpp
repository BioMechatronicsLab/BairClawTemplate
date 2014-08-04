
#include "BairClawInterface.h"
#include <math.h>

namespace barrett {

//##########################################################################
////////////////////////////////////////////////////////////////////////////
////////////////         BAIRCLAW Implementation          //////////////////
////////////////////////////////////////////////////////////////////////////
//##########################################################################

void BCInitThread( BCDigit* digit, const bool* initgoing) {
    
	char input;
	system("clear");
	printf("\nWould you like to run initialization? [y or n (use default ranges) ]: \n");
	input = getchar();
    
    printf("press [Enter] to continue to joint angle visualization.\n");

}
/** \desc BairClawVisThread need to be developed. It will provide an look into the system propties to be displyed periodically.
 */
void BairClawVisThread( BCDigit* digit, const bool* visgoing) {
 
}
    
    
#pragma mark - BCDigit

void BCDigit::setTendonForceOffset()
{
    //Models fit it matlab then copied into program
    mcpFestOffset = -5.971124e+02 + (+8.402597e-01 * FEmotor.A2) + (-4.617917e-02 * FEmotor.A1) + (-1.800978e-02 * PIPmotor.A2) + (-5.823183e-02 * PIPmotor.A1);
    mcpEestOffset = -1.753974e+03 + (-1.241008e-02 * FEmotor.A2) + (+2.670576e+00 * FEmotor.A1) + (-3.922166e-02 * PIPmotor.A2) + (-1.786393e-01 * PIPmotor.A1);
    pipFestOffset = -8.943469e+02 + (-3.292929e-02 * FEmotor.A2) + (-2.342654e-02 * FEmotor.A1) + (+9.952569e-01 * PIPmotor.A2) + (-5.316950e-02 * PIPmotor.A1);
    pipEestOffset = -2.949234e+03 + (-9.191377e-02 * FEmotor.A2) + (-1.433349e-01 * FEmotor.A1) + (+8.367845e-02 * PIPmotor.A2) + (+3.056697e+00 * PIPmotor.A1);
}
    
    
void BCDigit::calcTendonForce()
{
    //Models fit it matlab then copied into program
    //If you don't want baseline values from the SG equation then don't cal setTedonForceOffset priot to calling calcTendonForce
    mcpFest = (-5.971124e+02 + (+8.402597e-01 * FEmotor.A2) + (-4.617917e-02 * FEmotor.A1) + (-1.800978e-02 * PIPmotor.A2) + (-5.823183e-02 * PIPmotor.A1)) - (mcpFestOffset);
    mcpEest = (-1.753974e+03 + (-1.241008e-02 * FEmotor.A2) + (+2.670576e+00 * FEmotor.A1) + (-3.922166e-02 * PIPmotor.A2) + (-1.786393e-01 * PIPmotor.A1)) - (mcpEestOffset);
    pipFest = (-8.943469e+02 + (-3.292929e-02 * FEmotor.A2) + (-2.342654e-02 * FEmotor.A1) + (+9.952569e-01 * PIPmotor.A2) + (-5.316950e-02 * PIPmotor.A1)) - (pipFestOffset);
    pipEest = (-2.949234e+03 + (-9.191377e-02 * FEmotor.A2) + (-1.433349e-01 * FEmotor.A1) + (+8.367845e-02 * PIPmotor.A2) + (+3.056697e+00 * PIPmotor.A1)) - (pipEestOffset);
    
    
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

void BCDigit::calcPercentage(){
    
     ADABRange = (double)ADABmax - (double)ADABmin;
     FERange   = (double)FEmax   - (double)FEmin;
     PIPRange  = (double)PIPmax  - (double)PIPmin;
     DIPRange  = (double)DIPmax  - (double)DIPmin;
     
     jointPercent[0] = 100 - ( ((double)ADABRange - ((double)jointVal[0] - (double)ADABmin)) / (double)ADABRange ) *100;
     jointPercent[1] =    ( ((double)FERange   - ((double)jointVal[1]   - (double)FEmin))   / (double)FERange )   *100;
     jointPercent[2] =  100 - ( ((double)PIPRange  - ((double)jointVal[2]  - (double)PIPmin))  / (double)PIPRange )  *100;
     jointPercent[3] =   ( ((double)DIPRange  - ((double)jointVal[3]  - (double)DIPmin))  / (double)DIPRange )  *100;
 
     //Tie percentages and joint angles together for debuging purposes.
     calcJointAngles();
}
    
void BCDigit::calcJointAngles(){
    
    scaledJointVal[0] = ((jointVal[0] - 0.5000000000) / 0.7071067812);
    jointValDeg[0] = ( 0.0000000000 * pow(scaledJointVal[0],2) ) + ( 0.7071067812 * pow(scaledJointVal[0],1) ) + ( 0.5000000000 * pow(scaledJointVal[0],0) );
    scaledJointVal[1] = ((jointVal[1] - 751.0000000000) / 212.8138858252);
    jointValDeg[1] = ( -1.9158678068 * pow(scaledJointVal[1],2) ) + ( -27.5322948607 * pow(scaledJointVal[1],1) ) + ( 51.7029936060 * pow(scaledJointVal[1],0) );
    scaledJointVal[2] = ((jointVal[2] - 366.3750000000) / 293.5229061395);
    jointValDeg[2] = ( -1.7749508464 * pow(scaledJointVal[2],2) ) + ( 24.5996263047 * pow(scaledJointVal[2],1) ) + ( 36.5530819906 * pow(scaledJointVal[2],0) );
    scaledJointVal[3] = ((jointVal[3] - 748.3750000000) / 79.9677390309);
    jointValDeg[3] = ( -5.4879315557 * pow(scaledJointVal[3],2) ) + ( -26.3204733439 * pow(scaledJointVal[3],1) ) + ( 39.8019401112 * pow(scaledJointVal[3],0) );
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
	
    printf("In setStaticFriction() enabled - %s \n",FEmotor.isEnabled ? "true":"false");
    //SET MCP F/E static values
    int initFE = jointVal[1];
    int demandCurrent = 40;
    if(FEmotor.isEnabled)
    {
        printf("Setting Flexion");
        while( abs(initFE - jointVal[1]) < 3 && demandCurrent <= 100)
        {
            printf("delta - %d demandCurrent - %d\n",abs(initFE - jointVal[1]), demandCurrent);
            FEmotor.SetCurrent(-demandCurrent++);
            btsleep(0.2);
        }
        FEmotor.SetCurrent(0);
        FEmotor.staticFrictionF = abs(demandCurrent);
    }else
    {
        printf("Can not set static friction. Motor must first be enabled");
    }
    
    demandCurrent = 40;
    FEmotor.SetCurrent(demandCurrent++);
    btsleep(0.5);
    
    if(FEmotor.isEnabled)
    {
        int initFE = jointVal[1];
        printf("Setting Extenstion \n");
        while( abs(initFE - jointVal[1]) < 3 && demandCurrent <= 100)
        {
            printf("delta - %d demandCurrent - %d\n",abs(initFE - jointVal[1]), demandCurrent);
            FEmotor.SetCurrent(demandCurrent++);
            btsleep(0.2);
        }
        FEmotor.SetCurrent(0);
        FEmotor.staticFrictionE = abs(demandCurrent);
    }
    

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
void BCHand::print(){
  //- NICE DISPLAY -----------------------------------------
    for(int i=0; i < digit.size(); i++)
    {
        digit[i].calcPercentage();
        system("clear");
        printf("digit[%d]\n - Joint Percentace - ", i);
        for(int j=0; j<4; j++)
        {
            printf("[%d]p-%4.2f v-%d ",j, digit[i].jointPercent[j], digit[i].jointVal[j]);
        }
        printf("(%d)\n",i);
        printf("FE-[ %d, %d] PIP [ %d, %d] \n", digit[i].FEmotor.A1,
                                                digit[i].FEmotor.A2,
                                                digit[i].PIPmotor.A1,
                                                digit[i].PIPmotor.A2);
    }
    usleep(100000);
    
}
    
    


    
    
    
} //From namespace barrett{}
