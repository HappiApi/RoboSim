#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "calcPos.h"

#define followSpeed 127 //15
#define frontIROffset 4
#define followSide LEFT
#define thresholdDist 35.0
#define TRACK 23.5
#define MIN_TURN_RADIUS 25

int leftEnc; int rightEnc;

void comeToStop() {
//  while (get_front_ir_dist(RIGHT) > 15) {
//    set_motors(15,15);
//  }
  set_motors(0,0);
  usleep(100000);
  get_motor_encoders(&leftEnc,&rightEnc);
  calcPosition(leftEnc,rightEnc);
  reset_motor_encoders();
  while (get_front_ir_dist(RIGHT) < 15) {
    set_motors(-15,-15);
  }
  get_motor_encoders(&leftEnc,&rightEnc);
  calcPosition(leftEnc,rightEnc);
  exit(0);
}

int main() {
	connect_to_robot();
	initialize_robot();
  set_origin();
  set_ir_angle(RIGHT, -45);
  int i = 0;

  int wallDist = get_front_ir_dist(LEFT);
  float turnRate = 0;
  float multiplierL = 0.5*followSpeed;
  float multiplierR = followSpeed;


  while (1) {
  	wallDist = get_front_ir_dist(LEFT);
    if (get_front_ir_dist(RIGHT) < 25) {
      comeToStop();
    }
  	if (wallDist == thresholdDist) {
  		set_motors(followSpeed, followSpeed);
  	} else if (wallDist < thresholdDist) {
  		turnRate = 1.0 - (0.8 * wallDist / thresholdDist);
  		set_motors(followSpeed, (int)(followSpeed - multiplierR * turnRate));  //Turn right
  	} else {
      if (wallDist > 70) {
        turnRate = 1.0;
      } else {
  	   	turnRate = 1.5 * (wallDist / thresholdDist - 1.0);
  		}
      set_motors((int)(followSpeed - multiplierL * turnRate), followSpeed);
  	}

    if (i % 8 == 0) {
      get_motor_encoders(&leftEnc,&rightEnc);
      calcPosition(leftEnc,rightEnc);
      reset_motor_encoders();
    }
    i++;
    usleep(10000);
  }

  return 1;
}