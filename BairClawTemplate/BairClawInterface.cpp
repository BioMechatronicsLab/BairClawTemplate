
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
    mcpFestOffset = 0.009806*(-5.971124e+02 + (+8.402597e-01 * FEmotor.A2) + (-4.617917e-02 * FEmotor.A1) + (-1.800978e-02 * PIPmotor.A2) + (-5.823183e-02 * PIPmotor.A1));
    mcpEestOffset = 0.009806*(-1.753974e+03 + (-1.241008e-02 * FEmotor.A2) + (+2.670576e+00 * FEmotor.A1) + (-3.922166e-02 * PIPmotor.A2) + (-1.786393e-01 * PIPmotor.A1));
    pipFestOffset = 0.009806*(-8.943469e+02 + (-3.292929e-02 * FEmotor.A2) + (-2.342654e-02 * FEmotor.A1) + (+9.952569e-01 * PIPmotor.A2) + (-5.316950e-02 * PIPmotor.A1));
    pipEestOffset = 0.009806*(-2.949234e+03 + (-9.191377e-02 * FEmotor.A2) + (-1.433349e-01 * FEmotor.A1) + (+8.367845e-02 * PIPmotor.A2) + (+3.056697e+00 * PIPmotor.A1));
}
    
    
void BCDigit::calcTendonForce()
{
    //Models fit it matlab then copied into program
    //If you don't want baseline values from the SG equation then don't cal setTedonForceOffset priot to calling calcTendonForce
    mcpFest = (0.009806*(-5.971124e+02 + (+8.402597e-01 * FEmotor.A2) + (-4.617917e-02 * FEmotor.A1) + (-1.800978e-02 * PIPmotor.A2) + (-5.823183e-02 * PIPmotor.A1))) - (mcpFestOffset);
    mcpEest = (0.009806*(-1.753974e+03 + (-1.241008e-02 * FEmotor.A2) + (+2.670576e+00 * FEmotor.A1) + (-3.922166e-02 * PIPmotor.A2) + (-1.786393e-01 * PIPmotor.A1))) - (mcpEestOffset);
    pipFest = (0.009806*(-8.943469e+02 + (-3.292929e-02 * FEmotor.A2) + (-2.342654e-02 * FEmotor.A1) + (+9.952569e-01 * PIPmotor.A2) + (-5.316950e-02 * PIPmotor.A1))) - (pipFestOffset);
    pipEest = (0.009806*(-2.949234e+03 + (-9.191377e-02 * FEmotor.A2) + (-1.433349e-01 * FEmotor.A1) + (+8.367845e-02 * PIPmotor.A2) + (+3.056697e+00 * PIPmotor.A1))) - (pipEestOffset);

    
}
    
void BCDigit::calcJacobianActuation()
{

    if( (jointValRad[1]*(180/M_PI)) <   15)
    {
        mcpScaledRadiusE = (( (jointValRad[1]*(180/M_PI)) - -2.5000000000) / 12.2474487139);
        mcpJointRadiusE = ( 0.0000422017 * pow(mcpScaledRadiusE,3) ) + ( -0.0001427911 * pow(mcpScaledRadiusE,2) ) + ( -0.0003308749 * pow(mcpScaledRadiusE,1) ) + ( 0.0100116810 * pow(mcpScaledRadiusE,0) );
    }
    else
    {
        mcpScaledRadiusE = (( (jointValRad[1]*(180/M_PI)) - 52.5000000000) / 23.8047614285);
        mcpJointRadiusE = ( 0.0000869093 * pow(mcpScaledRadiusE,3) ) + ( -0.0004046143 * pow(mcpScaledRadiusE,2) ) + ( -0.0011856969 * pow(mcpScaledRadiusE,1) ) + ( 0.0088798494 * pow(mcpScaledRadiusE,0) );
    }
    if( (jointValRad[1]*(180/M_PI)) <   25)
    {
        mcpScaledRadiusF = (( (jointValRad[1]*(180/M_PI)) - 2.5000000000) / 15.1382517705);
        mcpJointRadiusF= ( -0.0000462686 * pow(mcpScaledRadiusF,3) ) + ( -0.0002187015 * pow(mcpScaledRadiusF,2) ) + ( 0.0003988968 * pow(mcpScaledRadiusF,1) ) + ( 0.0094403190 * pow(mcpScaledRadiusF,0) );
    }
    else
    {
        mcpScaledRadiusF = (( (jointValRad[1]*(180/M_PI)) - 57.5000000000) / 20.9165006634);
        mcpJointRadiusF = ( -0.0001741799 * pow(mcpScaledRadiusF,5) ) + ( -0.0004295531 * pow(mcpScaledRadiusF,4) ) + ( -0.0004321000 * pow(mcpScaledRadiusF,3) ) + ( -0.0007281876 * pow(mcpScaledRadiusF,2) ) + ( -0.0002048097 * pow(mcpScaledRadiusF,1) ) + ( 0.0101367815 * pow(mcpScaledRadiusF,0) );
    }
    DHp.jacobianActuation(1,2) = mcpJointRadiusF;
    DHp.jacobianActuation(1,3) = -mcpJointRadiusE;
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
    jointValRad[0] = ( 0.0000000000 * pow(scaledJointVal[0],4) ) + ( 0.0000000000 * pow(scaledJointVal[0],3) ) + ( 0.0000000000 * pow(scaledJointVal[0],2) ) + ( 0.0123413415 * pow(scaledJointVal[0],1) ) + ( 0.0087266463 * pow(scaledJointVal[0],0) );
    //AdAbduction set to ZERO because not yet calibrated *******************************************************
    jointValRad[0] = 0;
    scaledJointVal[1] = ((jointVal[1] - 751.0000000000) / 212.8138858252);
    jointValRad[1] = ( -0.0348243796 * pow(scaledJointVal[1],4) ) + ( -0.0742307570 * pow(scaledJointVal[1],3) ) + ( 0.0147427679 * pow(scaledJointVal[1],2) ) + ( -0.3832364401 * pow(scaledJointVal[1],1) ) + ( 0.8952705639 * pow(scaledJointVal[1],0) );
    scaledJointVal[2] = ((jointVal[2] - 366.3750000000) / 293.5229061395);
    jointValRad[2] = ( -0.0254189471 * pow(scaledJointVal[2],4) ) + ( 0.0823974499 * pow(scaledJointVal[2],3) ) + ( -0.0038441207 * pow(scaledJointVal[2],2) ) + ( 0.3224575594 * pow(scaledJointVal[2],1) ) + ( 0.6355146797 * pow(scaledJointVal[2],0) );
    scaledJointVal[3] = ((jointVal[3] - 748.3750000000) / 79.9677390309);
    jointValRad[3] = ( -0.0703793344 * pow(scaledJointVal[3],4) ) + ( -0.1606000938 * pow(scaledJointVal[3],3) ) + ( -0.0562508357 * pow(scaledJointVal[3],2) ) + ( -0.2914736187 * pow(scaledJointVal[3],1) ) + ( 0.6969941865 * pow(scaledJointVal[3],0) );
}

void BCDigit::calcDHparams(){
    
    DHp.theta << jointValRad[0], jointValRad[1], jointValRad[2], jointValRad[3];
    DHp.calcT();
    DHp.calcJacobian();
    DHp.pinvJacobianTrans();
    

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
