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

struct coordinates 
{
  double x;
  double y;
};

struct coordinates coords; 

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
	coords.x += rightEncInMillimetres * sin(prevAngle) ;	
	coords.y += rightEncInMillimetres * cos(prevAngle) ;
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
		coords.x += deltaX ;
		coords.y += deltaY ;
	}

//Adjust current angle to previous angle

void changePrevAngle()
	{
		prevAngle += angleTurned ;	//adjusted to += 
	}

void calcPosition(int leftEnc, int rightEnc) 
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
		set_point((int)(coords.x / 10), (int)(coords.y / 10)); //set point in centimetres
	}
	printf("X : %f Y : %f Angle : %f\n", coords.x, coords.y, prevAngle * 180 / 3.141592);
	printf("Dist Travelled: %i, angle travelled at: %f\n", (int)sqrt((coords.x * coords.x) + (coords.y * coords.y)), 90 - atan(coords.y / coords.x) * 180 / 3.141592); 
}

