#include <stdio.h>
#include <math.h>
#include "picomms.h"

#define encToMillimetres 1000/1145
#define widthOfRobot 235		// In mm

double angleTurned;		// current turning angle
double prevAngle = 0;		// initial condition previous angle = 0
double deltaX, deltaY;
double Rl, Rr, Rm;
double leftEncInMillimetres, rightEncInMillimetres;
double p, q, a, b ; 	// Should probably find better name
double xCoord, yCoord;

void convertEncToMM(int leftEnc, int rightEnc)
	{
		leftEncInMillimetres = (double)leftEnc * encToMillimetres ;
		rightEncInMillimetres = (double)rightEnc * encToMillimetres ;
	}

void calcParameters()
	{
		angleTurned = ( leftEncInMillimetres - rightEncInMillimetres ) / widthOfRobot ;
		Rl = leftEncInMillimetres / angleTurned ;
		Rr = rightEncInMillimetres / angleTurned ;
		Rm = ( Rl + Rr ) / 2 ;
	}

void calcStraightDist()	// case where angleTurned == 0
{
	xCoord += rightEncInMillimetres * sin(prevAngle) ;	
	yCoord += rightEncInMillimetres * cos(prevAngle) ;
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
		xCoord += deltaX ;
		yCoord += deltaY ;
	}

//Adjust current angle to previous angle

void changePrevAngle()
	{
		prevAngle += angleTurned ;	//adjusted to += 
	}

void calcPosition(int leftEnc, int rightEnc, double* xCurrent, double* yCurrent) 
{
	convertEncToMM(leftEnc, rightEnc);
	if (fabs(leftEncInMillimetres - rightEncInMillimetres) < 1.0)
	{
		calcStraightDist();
	}
	else
	{
		calcParameters();
		calcDeltas();
		addDeltasToCumulative();
		changePrevAngle();
		set_point((int)(*xCurrent / 10), (int)(*yCurrent / 10)); //set point in centimetres
	}
	printf("X : %f Y : %f Angle : %f\n", *xCurrent, *yCurrent, prevAngle * 180 / 3.141592);
	printf("Dist Travelled: %imm, angle travelled at: %f degrees\n", (int)sqrt((*xCurrent * *xCurrent) + (*yCurrent * *yCurrent)), atan(*xCurrent / *yCurrent) * 180 / 3.141592); 
  *xCurrent = xCoord;
  *yCurrent = yCoord;
}

