#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "calcPos.h"

#define distanceToWall 20
#define distanceToFront 25
#define endOfWallMultiplier 2
#define slowSpeed 30 //15
#define fastSpeed 50 //30
#define superSpeed 70 //50
#define frontIROffset 4
#define followSide LEFT
#define threshold 5

int leftEnc,rightEnc;

void setSensors()
{
	set_ir_angle(LEFT, -45);
	set_ir_angle(RIGHT,-75);
}

// int angleToWall() {
// 	int frontR = get_front_ir_dist(RIGHT);
// 	int frontL = get_front_ir_dist(LEFT);
	
// }

void checkWallAhead() {
  int usDist = get_us_dist();

  if (usDist < 30 && get_front_ir_dist(RIGHT) > 35) {
    while (usDist > 15) {
      set_motors(10,10);
      usDist = get_us_dist();
    }
    printf("STOPPED");
    while(1){
      set_motors(0,0);
	}
  }
}

void adjustParallel()
{
	if (get_side_ir_dist(followSide) > get_front_ir_dist(followSide)-frontIROffset)		
	{
		set_motors(fastSpeed,slowSpeed);
		printf("RIGHT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
	}

	if(get_side_ir_dist(followSide) < get_front_ir_dist(followSide)-frontIROffset)
	{
		set_motors(slowSpeed,fastSpeed);
		printf("LEFT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
	}	
}

void lookForFront()
{
	if(get_front_ir_dist(RIGHT) < distanceToFront)
	{
		while(get_front_ir_dist(RIGHT) < distanceToFront)
		{
			set_motors(fastSpeed,-fastSpeed);
			printf(" FRONT FR%d\n",get_front_ir_dist(RIGHT) );

			checkWallAhead();

			get_motor_encoders(&leftEnc,&rightEnc);
			calcPosition(leftEnc,rightEnc);
			reset_motor_encoders();
		}
	}
}

void lookForEnd()
{
	if( endOfWallMultiplier * get_side_ir_dist(followSide) < get_front_ir_dist(followSide)-frontIROffset)
	{
		while(endOfWallMultiplier * get_side_ir_dist(followSide) < get_front_ir_dist(followSide))
		{
			set_motors(slowSpeed,superSpeed);
			printf("UTURN SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide) );

			checkWallAhead();

			get_motor_encoders(&leftEnc,&rightEnc);
			calcPosition(leftEnc,rightEnc);
			reset_motor_encoders();
		}
	}
}

void straight()
{
	if( get_side_ir_dist(followSide) == get_front_ir_dist(followSide)-frontIROffset)
	{
		set_motors(fastSpeed,fastSpeed);
		printf("STRAIGHT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
	}
}

/*void adjustSide()
{
	if(get_side_ir_dist(followSide) - distanceToWall > threshold || get_front_ir_dist(followSide) - distanceToWall > threshold)
	{
		while(get_side_ir_dist(followSide) - distanceToWall || get_front_ir_dist(followSide) < distanceToWall)
		{
			set_motors(fastSpeed,slowSpeed);
			printf(" SIDE OUT %d %d\n", get_side_ir_dist(LEFT), get_front_ir_dist(LEFT));
		}
	}

	if(get_side_ir_dist(followSide) > distanceToWall || get_front_ir_dist(followSide) > distanceToWall)
	{
		while(get_side_ir_dist(followSide) > distanceToWall || get_front_ir_dist(followSide) > distanceToWall)
		{
			set_motors(slowSpeed,fastSpeed);
			printf(" SIDE IN %d %d\n", get_side_ir_dist(LEFT), get_front_ir_dist(LEFT));
		}
	}
}*/

void adjustSide()
{
	if(abs(get_front_ir_dist(followSide) - distanceToWall) > threshold && abs(get_side_ir_dist(followSide) - distanceToWall) > threshold) // check over threshold
	{
		if(get_front_ir_dist(followSide) - distanceToWall < 0 && get_side_ir_dist(followSide) - distanceToWall < 0)	//case turn right
		{
			while(abs(get_front_ir_dist(followSide) - distanceToWall) > threshold && abs(get_side_ir_dist(followSide) - distanceToWall) > threshold)
			{
				set_motors(fastSpeed,slowSpeed);
				printf(" SIDE OUT %d %d\n", get_side_ir_dist(LEFT), get_front_ir_dist(LEFT));

				checkWallAhead();

				get_motor_encoders(&leftEnc,&rightEnc);
				calcPosition(leftEnc,rightEnc);
				reset_motor_encoders();
			}
		}

		if(get_front_ir_dist(followSide) - distanceToWall > 0 && get_side_ir_dist(followSide) - distanceToWall > 0) //case turn left
		{
			while(abs(get_front_ir_dist(followSide) - distanceToWall) > threshold && abs(get_side_ir_dist(followSide) - distanceToWall) > threshold)
			{
				set_motors(slowSpeed,fastSpeed);
				printf(" SIDE IN %d %d\n", get_side_ir_dist(LEFT), get_front_ir_dist(LEFT));

				checkWallAhead();

				get_motor_encoders(&leftEnc,&rightEnc);
				calcPosition(leftEnc,rightEnc);
				reset_motor_encoders();
			}
		}
	}
}

int main()
{
	connect_to_robot();
	initialize_robot();
	setSensors();
	set_origin();

	while(1)
	{
		adjustParallel();
		lookForFront();
		lookForEnd();
		straight();
		adjustSide();
		checkWallAhead();
		get_motor_encoders(&leftEnc,&rightEnc);
		calcPosition(leftEnc,rightEnc);
		reset_motor_encoders();
	}
}