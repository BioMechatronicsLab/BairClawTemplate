//
//  filters.h
//  BairClawTemplate
//
//  Created by Randy Hellman on 8/12/14.
//  Copyright (c) 2014 Randy Hellman. All rights reserved.
//

#ifndef __BairClawTemplate__filters__
#define __BairClawTemplate__filters__

#include <stdio.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace Eigen;

namespace barrett{
    
    class butterworthFilter
    {
        /* Example initialization of butterworth filter defined from matlab code
         
        double numBW[] = {   0.002898,   0.008695,   0.008695,   0.002898};
        double denBW[] = {   1.000000,  -2.374095,   1.929356,  -0.532075};
        barrett::butterworthFilter testFilter(3, numBW, denBW);
        */
    public:
        int order;
        double *num, *den, *y, *x;

        butterworthFilter(int orderBW, double numBW[], double denBW[])
        {
            order = orderBW+1; //add one to accout for the fact the order has one additional term
            num = new double[order];
            den = new double[order];
            y   = new double[order];
            x   = new double[order];
            for (int i=0; i<order; i++)
            {
                num[i] = numBW[i];
                den[i] = denBW[i];
                x[i]   = 0;
                y[i]   = 0;
            }
        }
        
        ~butterworthFilter(){
            delete []den;
            delete []num;
            delete []x;
            delete []y;
        }
        
        double update(double input)
        {
            double numSum = 0;
            double denSum = 0;
            //shift x and y vectors to set new state
            for(int i=order-1; i>0; i--)
            {
                x[i] = x[i-1];
                y[i] = y[i-1];
            }
            
            x[0] = input;
            for(int i=0; i<order; i++)
            {
                numSum += num[i]*x[i];
                if(i != 0)
                {
                denSum += den[i]*y[i];
                }
            }
            y[0] = (numSum - denSum) / den[0];
            
            return y[0];
        }
        
    };
    
/* Example matlab code to determine butterworth filter parameters
 
 % Randy Hellman
 % Pdc lowpass filter
 close all
 clear
 cutOffFreq = 10;
 samplingRate = 200;
 Ts = 1/samplingRate;
 Wn = cutOffFreq / (samplingRate/2);
 filterOrder = 3;
 [num den] = butter(filterOrder,Wn,'low');
 h = fvtool(num, den);
 
 x = 1:0.005:2;
 xN = x;
 xN(end-(round(length(x)/2)):10:end-(round(length(x)/3))) = 10;
 
 y = filter(num ,den,xN);
 plot(xN,'r')
 hold on
 plot(y,'ob')
 yF = zeros(length(xN),1);
 for i=4:length(xN)
 yF(i) = (num * [x(i); x(i-1); x(i-2); x(i-3)] - den(2)*y(i-1) - den(3)*y(i-2) -den(4)*y(i-3))/den(1);
 end
 
 plot(yF,'.r')
 
 fprintf('\ndouble numBW[] = {')
 for i=1:(filterOrder)
 fprintf(' %10.6f,', num(i))
 end
 fprintf(' %10.6f};',num(i+1))
 
 fprintf('\ndouble denBW[] = {')
 for i=1:(filterOrder)
 fprintf(' %10.6f,', den(i))
 end
 fprintf(' %10.6f};\n',den(i+1))
 
 fprintf('double xN[] = {')
 for i=1:length(xN)
 fprintf(' %10.6f,',xN(i))
 end
 fprintf('};')
 
 */


}; //From namespace barrett{}
#endif /* defined(__BairClawTemplate__filters__) */
