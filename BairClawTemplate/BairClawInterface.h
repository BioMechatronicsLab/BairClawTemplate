//
//  BairClawInterface.h
//  
//
//  Created by Randy Hellman on 10/31/13.
//
//

#ifndef _BairClawInterface_h
#define _BairClawInterface_h
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <vector>
#include <string>
#include <cstdio>
#include <math.h>

#include <boost/ref.hpp>
#include <boost/bind.hpp>
#include <boost/tuple/tuple.hpp>
#include <unistd.h>
#include <barrett/os.h>  // For btsleep()
#include <boost/thread.hpp>
#include <barrett/bus/can_socket.h>

#include <barrett/detail/stl_utils.h>  // waitForEnter()
#include <barrett/math.h>
#include <barrett/units.h>
#include <barrett/systems.h>
#include <barrett/log.h>
#include <barrett/products/product_manager.h>

#include "EPOSInterface.h" //EPOS control outside of digit control


#include <Eigen/Dense>
#include <Eigen/Geometry>




using namespace Eigen;

namespace barrett{
    
template <typename DerivedA, typename DerivedB>
void DH2T( MatrixBase<DerivedA>& DH, MatrixBase<DerivedB>& T);

    
#pragma mark - BioMechMatrixManip
#define NUM_LINKS       4
#define LINK_1          0.0422
#define LINK_1_OFFSET  -0.0128
#define LINK_2          0.0318
#define LINK_3          0.0200
#define PULLEY_1        0.009525
#define PULLEY_2        100 //changes based on theta to be reassigned
#define PULLEY_3        0.006223
#define PULLEY_4        0.006350
    
    
class DHparams
{
public: //remove later once debuging
    VectorXd theta;
    MatrixXd jacobian;
    /**
     * jacobianPseudoInvers null until calcJacobian and pinvJacobian are called
     */
    MatrixXd jacobianTransposePseudoInverse;
    MatrixXd jacobianActuation;
    MatrixXd tendonForce;
    MatrixXd endEffectorForce;
    MatrixXd endEffectorStep01;
    
    std::vector<MatrixXd> transformationMatrixBetweenLink;
    std::vector<MatrixXd> transformationMatrixToGlobal;
    
    std::vector<VectorXd> DH;

    
    DHparams()
    {
        jacobian.resize(6,NUM_LINKS);
        jacobian.Zero(6,NUM_LINKS);
        jacobianTransposePseudoInverse.resize(6, NUM_LINKS);
        jacobianTransposePseudoInverse.Zero(6, NUM_LINKS);
        jacobianActuation.resize(NUM_LINKS, 6);
        jacobianActuation.Zero(NUM_LINKS, 6);
        tendonForce.resize(6,1);
        tendonForce.Zero(6,1);
        endEffectorForce.resize(6,1);
        endEffectorForce.Zero(6,1);
        endEffectorStep01.resize(4,1);
        endEffectorStep01.Zero(4,1);
        
    
        theta.resize(4);
        theta << 0,0,0,0;
        theta(0) = 1.3;
        for(int i=0; i<NUM_LINKS; i++)
        {
            transformationMatrixBetweenLink.push_back(MatrixXd::Identity(4,4));
            transformationMatrixToGlobal.push_back(MatrixXd::Identity(4,4));
            DH.push_back(VectorXd::Zero(4));
        }
        //Assemble DH parameteres from initial theta values
        DH[0] <<      0,  -M_PI/2,             0, theta(0);
        DH[1] << LINK_1,        0, LINK_1_OFFSET, theta(1);
        DH[2] << LINK_2,        0,             0, theta(2);
        DH[3] << LINK_3,        0,             0, theta(3);
        
        jacobianActuation <<
            PULLEY_1, -PULLEY_1,        0,        0,        0,        0,
                   0,         0, PULLEY_2,-PULLEY_2,        0,        0,
                   0,         0,        0,        0, PULLEY_3,-PULLEY_3,
                   0,         0,        0,        0, PULLEY_4,-PULLEY_4;
        
        
    }
    
    /**
     * Computes all transformation matrices from the updated theat(input)
     */
    void calcT()
    {
        
        for(int i=0; i<NUM_LINKS; i++)
        {
            DH[i](3) = theta(i);
        }
        
        for(int i=0; i<NUM_LINKS; i++)
        {//computes tranformation matrices from DH paramters
            DH2T(DH[i], transformationMatrixBetweenLink[i]);
        }
        
        for(int i=0; i<NUM_LINKS; i++)
        {// calculates each transformation from current frame to global
            if(i == 0)
            {
                transformationMatrixToGlobal[i] = transformationMatrixBetweenLink[i];
            }
            else
            {
                transformationMatrixToGlobal[i] = transformationMatrixToGlobal[i-1] * transformationMatrixBetweenLink[i];
            }
        }
    }
    /** \returns
     *\note Function to create Jacobial 6 x n where n is the number of links
     * Ti = 6x n*4 matrix that is a list of R01,R20....,Rn0
     *   Jvi = { Zi-1 X (On-Oi-1)   revolute
     *         {  Zi-1              prismatic
     *   Jwi = { Zi-1               revolute
     *         {  0                 prismatic
     * ------------- JACOBIAN ONLY GOOD FOR ROTATIONAL JOINTS ----------------
     */
    void calcJacobian()
    {
        static Vector3d z_pre, o_pre, crossP;
        z_pre  << 0, 0, 1;
        o_pre  << 0, 0, 0;
        crossP << 0, 0, 0;
        
        
        for(int i=0; i<NUM_LINKS; i++)
        {
            if(i > 0)
            {
                z_pre = transformationMatrixToGlobal[i-1].block(0,2,3,1);
                o_pre = transformationMatrixToGlobal[i-1].block(0,3,3,1);
            }
            jacobian.block(0,i,3,1) = z_pre.cross((Vector3d)transformationMatrixToGlobal.back().block(0,3,3,1) - o_pre); // Vi
            jacobian.block(3,i,3,1) = z_pre; // Wi
        }
    }
    
    void pinvJacobianTrans()
    {
        static const double  pinvtoler=1.0e-6; // choose your tolerance wisely!
        VectorXd singularValuesM(NUM_LINKS);
        JacobiSVD<MatrixXd> svd(jacobian.transpose(), ComputeThinU | ComputeThinV);
        
        for(int i=0; i<NUM_LINKS; i++)
        {
            if ( svd.singularValues()(i) > pinvtoler )
                singularValuesM(i)= 1.0/svd.singularValues()(i);
            else singularValuesM(i)=0;
        }
        jacobianTransposePseudoInverse = (svd.matrixV() * singularValuesM.asDiagonal() * svd.matrixU().transpose());
        
        
    }
    
};
    

#pragma mark - BCDigit
class BCDigit {
	int setCounter;
	int JointShutOffRange;
public:
	EPOS2 FEmotor, PIPmotor, ADABmotor;
    int node;
	bool isInit;
	int adab, fe, pip, dip;
    int jointVal[4];
    double jointPercent[4], jointValRad[4], scaledJointVal[4];
	int ADABmin, FEmin, PIPmin, DIPmin;
	int ADABmax, FEmax, PIPmax, DIPmax;
	double ADABRange, FERange, PIPRange, DIPRange;
    double mcpFest, mcpEest, pipFest, pipEest, mcpFestOffset, mcpEestOffset, pipFestOffset, pipEestOffset;
    double mcpScaledRadiusE, mcpJointRadiusE, mcpScaledRadiusF, mcpJointRadiusF;
    std::string description; //assined to BCdigit as a descrpiter
    DHparams DHp;
    // Static member variables & functions //

    

    
    // Constructor. Minimal error checking be carefule and make sure you
    // know what you are doing!!
	BCDigit(int node, const bus::CANSocket* busSet): FEmotor( node, busSet), PIPmotor( node+1, busSet), ADABmotor( node+2, busSet), node(node)
	{
		adab = 0; ADABmin = 18 ; ADABmax = 1020;
		fe   = 0; FEmin   = 480; FEmax   = 1021;
		pip  = 0; PIPmin  = 2  ; PIPmax  = 790;
		dip  = 0; DIPmin  = 611; DIPmax  = 832;
		setCounter = 0;
		isInit = 0;
		JointShutOffRange = 5;
		
        
        //Initialize tendonForce Variables
        mcpFest = 0; mcpEest = 0; pipFest = 0; pipEest = 0;
        mcpFestOffset = 0; mcpEestOffset = 0; pipFestOffset = 0; pipEestOffset = 0;
        mcpScaledRadiusE = 0; mcpJointRadiusE = 0; mcpScaledRadiusF = 0; mcpJointRadiusF = 0;
        description = "this is a bairClawDigit object";
        
        
	}
    
    /** \returns No return set jointVal[] property of class BCDigit
     *  \note needs to be called each time CAN dat frame is recieved to set data[8] to joint values
     */
	void set(unsigned char data[]){
		jointVal[0] = data[0] + (data[1] << 8);
		jointVal[1] = data[2] + (data[3] << 8);
		jointVal[2] = data[4] + (data[5] << 8);
		jointVal[3] = data[6] + (data[7] << 8);
	}
    void setTendonForceOffset();
    void calcTendonForce();
    void calcJacobianActuation();
	void init();
	void vis ();
	void calcPercentage();
    void calcJointAngles();
    void calcDHparams();
    void calcEndEffectorForce();
	void setStaticFriction();
	void backDrive();
    
    
	/*
     int  limitsOk(){
     
     int limit=1;
     if( (AdAb > AdAbmax-JointShutOffRange) || (AdAb < AdAbmin+JointShutOffRange) ){
     limit =0;
     }else if( (FE > FEmax-JointShutOffRange) || (FE < FEmin+JointShutOffRange) ){
     limit =0;
     }else if( (PIP < PIPmin+JointShutOffRange) && (DIP > DIPmax-JointShutOffRange) ){
     limit =0;
     }else if( (PIP > PIPmax-JointShutOffRange) && (DIP < DIPmin+JointShutOffRange) ){
     limit =0;
     }
     return limit;
     } */
	void print(){
        std::cout << description << std::endl;
	}
};


#pragma mark - BCHand
/**
 * \desc BCHand uses BCDigit to build a reference to complete hand. It also inherets from EPOS interface.
 */ 
class BCHand
{
public:
    std::vector<BCDigit> digit; //BCDigitPointer
    std::string name;

    
    BCHand(int NumberOfDigits, const bus::CANSocket* busSet, int startingNode=1)
    {
        for( int i=0; i<startingNode; i++ )
        {
            digit.push_back(BCDigit(startingNode+i, busSet));
        }
        
    }
    void print(); //Displays bairclaw data on screen at ~10Hz not to be called from a realtime thread! 
    
};

    
template <typename DerivedA, typename DerivedB>
void DH2T( MatrixBase<DerivedA>& DH, MatrixBase<DerivedB>& T)
{
    if( (DH.size() == 4))
    {
        /*T=[ cos(theta)  -sin(theta)*cos(alpha)  sin(theta)*sin(alpha) a*cos(theta);...
         sin(theta)   cos(theta)*cos(alpha) -cos(theta)*sin(alpha) a*sin(theta);...
         0            sin(alpha)             cos(alpha)            d           ;...
         0            0                      0                    1           ]; */
        double a = DH(0), alpha = DH(1), d = DH(2), theta = DH(3);
        
        T(0,0) = cos(theta);
        T(0,1) = -sin(theta)*cos(alpha);
        T(0,2) = sin(theta)*sin(alpha);
        T(0,3) = a*cos(theta);
        
        T(1,0) = sin(theta);
        T(1,1) = cos(theta)*cos(alpha);
        T(1,2) = -cos(theta)*sin(alpha);
        T(1,3) = a*sin(theta);
        
        T(2,0) = 0;
        T(2,1) = sin(alpha);
        T(2,2) = cos(alpha);
        T(2,3) = d;
        
        T(3,0) = 0;
        T(3,1) = 0;
        T(3,2) = 0;
        T(3,3) = 1;
    }
    else
    {
        printf("DH2T dimensions are off\n");
    }
    
}
    
    
    
    
}; //From namespace barrett{}
#endif
