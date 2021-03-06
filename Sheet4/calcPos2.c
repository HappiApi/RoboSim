/*
Module User Manual

Notes :

All distances are in mm
All angles are in radians

Prerequisites : Have to set_origin() before control loop
				Have to reset_motor_encoders() after every call
				Needs to be passed leftEnc & rightEnc as parameters to calcPosition()
			
TODO :
1. Remove reset_encoder dependency (Using previous encTick and currentEncTick)

*/


#include <stdio.h>
#include <math.h>
#include "picomms.h"

#define encToMillimetres 956/1000 //1000/956 // mm per ticks 	
#define widthOfRobot 236		// In mm

double angleTurned ;			// current turning angle in each call
double prevAngle = 0 ;		// initial condition previous angle = 0
double totalAngle = 0 ;		// cumulative angle turned
double deltaX, deltaY ;
double Rl, Rr, Rm ;			
double travelledLeftEncInMillimetres, travelledRightEncInMillimetres ;
double p, q, a, b ; 					// Should probably find better name
int prevLeftEnc, prevRightEnc = 0 ; 		// First loop previous = 0
int currentLeftEnc, currentRightEnc ;
int travelledLeftEnc, travelledRightEnc ;

struct coordinates 
{
  double x;
  double y;
};

struct coordinates coords; 

void convertEncToMM(int travelledLeftEnc, int travelledRightEnc)
{
	travelledLeftEncInMillimetres = (double)travelledLeftEnc * encToMillimetres ;
	travelledRightEncInMillimetres = (double)travelledRightEnc * encToMillimetres ;
}

void calcParameters()
{
	angleTurned = ( travelledLeftEncInMillimetres - travelledRightEncInMillimetres ) / widthOfRobot ;
	Rl = travelledLeftEncInMillimetres / angleTurned ; //no fabs
	Rr = travelledRightEncInMillimetres / angleTurned ;
	Rm = ( Rl + Rr ) / 2.0 ;
}

void calcStraightDist()	// case where angleTurned == 0
{
	coords.x += travelledRightEncInMillimetres * sin(totalAngle) ;	
	coords.y += travelledRightEncInMillimetres * cos(totalAngle) ;
}

//When robot !at 0 rad (given previous angle phi)  // <-- General case ?

void calcDeltas()
{
	p = Rm * sin(prevAngle + angleTurned) ;
	q = Rm * sin(prevAngle) ;
	deltaY = p - q ;

	//a = Rm - ( Rm * cos(prevAngle + angleTurned) ) ; //v1
	//b = Rm - ( Rm * cos(prevAngle) ) ;
	a = Rm * cos(prevAngle) ;		 //v2
	b = Rm * cos(prevAngle + angleTurned) ;
	deltaX = a - b ;
}

//Add deltaX and deltaY to cumulative position

void addDeltasToCumulative()
{
	coords.x += deltaX ;
	coords.y += deltaY ;
}

// Add Angle Turned to Cumulative Angle

void addAngleToCumulative()
{
	totalAngle += angleTurned ;
}

//Adjust current angle to previous angle

void changePrevAngle()
{
	prevAngle = angleTurned ;	//adjusted not += 
}

void printAndSetPoint()
{
	printf("X : %f cm Y : %f Angle cm : %f Degrees \n", coords.x/10, coords.y/10, totalAngle * 180 / 3.141592);
	set_point((int)(coords.x/10), (int)(coords.y/10));
}

void calcTravelledEnc(int leftEnc, int rightEnc)
{
	currentRightEnc = rightEnc ;
	currentLeftEnc = leftEnc ; 
	travelledRightEnc = currentRightEnc - prevRightEnc ;
	travelledLeftEnc = currentLeftEnc - prevLeftEnc ;
}

void changePrevEnc()
{
	prevRightEnc = currentRightEnc ;
	prevLeftEnc = currentLeftEnc ;
}


void calcPosition(int leftEnc, int rightEnc) 
{
	
	calcTravelledEnc(leftEnc, rightEnc) ;
	convertEncToMM(travelledLeftEnc, travelledRightEnc);

	if (fabs(travelledLeftEncInMillimetres - travelledRightEncInMillimetres) < 1.0)
	//if(leftEnc == rightEnc)
	{
		calcStraightDist();
		printf("In Straight\n");
	}
	else
	{
		calcParameters();
		calcDeltas();
		addDeltasToCumulative();
		addAngleToCumulative();
		changePrevAngle();
		printf("In Turn\n");
		
	}
	changePrevEnc() ;
	printAndSetPoint() ;



/*
When robot at 0 rad [Eg. Starting] use 
	{
		deltaY = Rm * math.sin(angleTurned) ;
		deltaX = Rm - ( Rm * math.cos(angleTurned) ) ;
	}
*/
}

