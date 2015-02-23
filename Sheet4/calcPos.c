#include <stdio.h>
#include <math.h>
#include "picomms.h"

#define encToCentimetres 1000/1145
#define widthOfRobot 225		// In mm

double angleTurned;		// current turning angle
double prevAngle = 0;		// initial condition previous angle = 0
double deltaX, deltaY;
double Rl, Rr, Rm;
double p, q, a, b ; 	// Should probably find better name
double xCoord, yCoord;

void calcParameters(int leftEnc, int rightEnc)
	{
		angleTurned = (double)( leftEnc - rightEnc ) / widthOfRobot ;
		Rl = (double)leftEnc / angleTurned ;
		Rr = (double)rightEnc / angleTurned ;
		Rm = ( Rl + Rr ) / 2 ;
	}

void calcStraightDist(int leftEnc, int rightEnc)	// case where angleTurned == 0
{
	xCoord += (double)leftEnc * encToCentimetres * sin(prevAngle) ;	
	yCoord += (double)rightEnc * encToCentimetres * cos(prevAngle) ;
}

//When robot !at 0 rad (given previous angle phi)  // <-- General case ?
void calcDeltas()
	{
		p = Rm * sin(prevAngle + angleTurned) ;
		q = Rm * sin(prevAngle) ;
		deltaY = p - q ;

		a = Rm - ( Rm * cos(prevAngle + angleTurned) ) ;
		b = Rm - ( Rm * cos(prevAngle) ) ;
		deltaX = a - b ;
	}

//Add deltaX and deltaY to cumulative position

void addDeltasToCumulative()
	{
		xCoord += deltaX * encToCentimetres ;
		yCoord += deltaY * encToCentimetres ;
	}

//Adjust current angle to previous angle

void changePrevAngle()
	{
		prevAngle += angleTurned ;	//adjusted to += 
	}

void calcPosition(int leftEnc, int rightEnc, double* xCurrent, double* yCurrent) 
{
	// convertEncToMM(leftEnc, rightEnc);
	if (fabs(leftEnc - rightEnc) < 1.0)
	{
		calcStraightDist(leftEnc, rightEnc);
	}
	else
	{
		calcParameters(leftEnc, rightEnc);
		calcDeltas();
		addDeltasToCumulative();
		changePrevAngle();
	}
	log_trail();
	set_point((int)(*xCurrent / 10), (int)(*yCurrent / 10)); //set point in centimetres
	printf("X : %f Y : %f Angle : %f\n", *xCurrent, *yCurrent, prevAngle * 180 / 3.141592);
	printf("Dist Travelled: %imm, angle travelled at: %f degrees\n", (int)sqrt((*xCurrent * *xCurrent) + (*yCurrent * *yCurrent)), atan(*xCurrent / *yCurrent) * 180 / 3.141592); 
  *xCurrent = xCoord;
  *yCurrent = yCoord;
}

