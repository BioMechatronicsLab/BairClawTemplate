//
//  codeSnidbits.h
//  BairClawTemplate
//
//  Created by Randy Hellman on 12/3/13.
//  Copyright (c) 2013 Randy Hellman. All rights reserved.
//

#ifndef BairClawTemplate_codeSnidbits_h
#define BairClawTemplate_codeSnidbits_h


    bairClaw.digit[0].FEmotor.SetCurrentLimit(300);
	bairClaw.digit[0].FEmotor.enable();
    bairClaw.digit[0].FEmotor.SetPositionProfile(500,1000,1000);
	bairClaw.digit[0].FEmotor.ActivateProfilePositionMode();

    bairClaw.digit[0].PIPmotor.SetCurrentLimit(300);
	bairClaw.digit[0].PIPmotor.enable();
    bairClaw.digit[0].PIPmotor.SetPositionProfile(300,1000,1000);
	bairClaw.digit[0].PIPmotor.ActivateProfilePositionMode();


    printf("Press [Enter] to rotate FE motor 2000");
	fflush(stdout);
	getchar();
    bairClaw.digit[0].FEmotor.MoveToPosition(1000, 1);
    printf("Press [Enter] to rotate PIP motor 2000");
	fflush(stdout);
	getchar();
    bairClaw.digit[0].PIPmotor.MoveToPosition(1000, 1);
int count=0, sw=0, i=0, success=1;
double FEdesiredPos = 5, PIPDIPdesiredPos = 10;
double changeInMotorPosFE = 0, changeInMotorPosPD = 0;


    while (count < 3000)
	{
		bairClaw.digit[0].calcPercentage();

        changeInMotorPosFE = (bairClaw.digit[0].jointPercent[1] - FEdesiredPos) * 50;
		changeInMotorPosPD = ((bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3]) - PIPDIPdesiredPos) * 50;

        if(count % 2 == 0)
        {
            bairClaw.digit[0].FEmotor.MoveToPosition(changeInMotorPosFE, 1);
        }
		else{
			bairClaw.digit[0].PIPmotor.MoveToPosition(changeInMotorPosPD, 1);
        }





		if(count % 5 == 0){ //if set to 1 does nothing just left in for quick changes
            success = system("clear");
            printf("Joint Percentace - ");
            for(j=0; j<4; j++)
            {
                printf("[%d]p-%4.2f v-%d ",j, bairClaw.digit[0].jointPercent[j],bairClaw.digit[0].jointVal[j]);
            }
            printf("(%d)\n",i);
            printf("FE-[ %d, %d] PIP [ %d, %d] \n",bairClaw.digit[0].FEmotor.A1,
                   bairClaw.digit[0].FEmotor.A2,
                   bairClaw.digit[0].PIPmotor.A1,
                   bairClaw.digit[0].PIPmotor.A2);
			printf("\nchangeInMotorPosFE = %6.2f, desiredPos = %4.2f, indexFinger.FEPercent = %4.2f count = %d\n",changeInMotorPosFE, FEdesiredPos, bairClaw.digit[0].jointPercent[1], count);
			printf("\nchangeInMotorPosPD = %6.2f, desiredPos = %4.2f, PIPPercent+DIPPercent = %4.2f count = %d\n",changeInMotorPosPD, PIPDIPdesiredPos, bairClaw.digit[0].jointPercent[2] + bairClaw.digit[0].jointPercent[3], count);
		}
		if(count % 100 == 0){
            if(count < 1300)
            {
                sw++;
                if(sw % 2 == 0 )
                {
                    FEdesiredPos = 35;
                    PIPDIPdesiredPos = 65;
                }else
                {
                    FEdesiredPos = 5;
                    PIPDIPdesiredPos = 10;
                }
            }else if( count < 1500)
            {
                sw++;
                if(sw % 2 == 0 )
                {
                    FEdesiredPos = 5;
                    PIPDIPdesiredPos = 15;
                }else
                {
                    FEdesiredPos = 5;
                    PIPDIPdesiredPos = 15;
                }
            }else if (count < 2500)
            {
                sw++;
                if(sw % 2 == 0 )
                {
                    FEdesiredPos = 35;
                    PIPDIPdesiredPos = 10;
                }else
                {
                    FEdesiredPos = 45;
                    PIPDIPdesiredPos = 10;
                }
            }else
            {
                    FEdesiredPos = 5;
                    PIPDIPdesiredPos = 10;
            }
        }
        count++;
        usleep(5000);
	}



#endif
