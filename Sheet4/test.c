#include <stdio.h>
#include <math.h>
#include "picomms.h"
#include "calcPos2.h"

int leftEnc,rightEnc ;

int main()
{
	connect_to_robot();
	initialize_robot();
	set_origin();
	set_ir_angle(RIGHT, -45);

	while(1)
	{
		if(get_front_ir_dist(LEFT) < 25)
		{
			set_motors(40,20);
		} 

		if(get_front_ir_dist(LEFT) > 25)
		{
			set_motors(20,40);
		} 
		get_motor_encoders(&leftEnc,&rightEnc);
		calcPosition(leftEnc,rightEnc);
		reset_motor_encoders();
	}

	return 1;
}