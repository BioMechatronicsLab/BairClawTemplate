//
//  BairClawMCPActuationJ.h
//
//  Created by Randy Hellman on 5/21/14.
//  Copyright (c) 2014 Randy Hellman. All rights reserved.
//

#ifndef bairClawTemplate_BairClawMCPActuationJ_h
#define bairClawTemplate_BairClawMCPActuationJ_h

#include <map>
#include <cmath>
#include <ctime>



double sign(double test);

std::map<double, double> initMCPMomentFlex();
std::map<double, double> initMCPMomentExt();

/**
 *  \desc MCPActuationRadius is a class that finds tabluated values for 
 *
 *  NEEDED to construct matrix
 *  MCPActuationRadius mcptest;
 *  mcptest.flextionMap  = &flextionMap;
 *  mcptest.extentionMap = &extentionMap;
 *  \returns No return set jointVal[] property of class BCDigit
 */
class MCPActuationRadius{

public:
    
    /** \desc extensionMap is pointer to the map of joint angles to exptension moment arm for the F/E MCP.
     */
    std::map<double, double> extensionMap;
    /** \desc flexionMap is pointer to the map of joint angles to flextion moment arm for the F/E MCP.
     */
    std::map<double, double> flexionMap;
    MCPActuationRadius()
    {
        extensionMap = initMCPMomentExt();
        flexionMap   = initMCPMomentFlex();
        
    }
    double flexionRadius(double jointAngle)
    {
        double static index = (int)jointAngle;
        double static indexDec = jointAngle - (int)jointAngle;

        if( (indexDec) > .75) //abs
        {
            indexDec = 1;
        }else if( ( (indexDec)) < .25 ) //abs
        {
            indexDec = 0;
        }else
        {
            indexDec = 0.5;
        }
        index = index + sign(jointAngle)*indexDec;
        
        if( flexionMap.find(index) != flexionMap.end())
        {
            return flexionMap.find(index)->second;
        }else
        {
            return 0;
        }
    }
    double extensionRadius(double jointAngle)
    {
        double static index = (int)jointAngle;
        double static indexDec = jointAngle - (int)jointAngle;
        
        if( (indexDec) > .75) //abs
        {
            indexDec = 1;
        }else if( ( (indexDec)) < .25 ) // abs
        {
            indexDec = 0;
        }else
        {
            indexDec = 0.5;
        }
        index = index + sign(jointAngle)*indexDec;
        
        if( extensionMap.find(index) != extensionMap.end())
        {
            return extensionMap.find(index)->second;
        }else
        {
            return 0;
        }
        
    }
    
};


#endif
