#include <stdio.h>
#include <math.h>

#define encToMillimetres 1000/956	
#define widthOfRobot 236		// In mm

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
		prevAngle += angleTurned ;	//adjusted
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
		set_origin();
		set_point((int)(coords.x / encToMillimetres), (int)(coords.y / encToMillimetres));
	}
	printf("X : %f Y : %f Angle : %f\n", coords.x, coords.y, angleTurned * 180 / 3.141592);
						
/*
When robot at 0 rad [Eg. Starting] use 
	{
		deltaY = Rm * math.sin(angleTurned) ;
		deltaX = Rm - ( Rm * math.cos(angleTurned) ) ;
	}
*/
}

