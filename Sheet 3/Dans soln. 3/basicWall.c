#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "calcPos.h"

#define followSpeed 30 //15
#define frontIROffset 4
#define followSide LEFT
#define thresholdDist 35.0
#define TRACK 23.5
#define MIN_TURN_RADIUS 25

int leftEnc; int rightEnc;

int main() {
	connect_to_robot();
	initialize_robot();
  set_origin();
  set_ir_angle(RIGHT, -45);
  int i = 0;

  float distToWall = thresholdDist * sin (3.1415 * 0.25);
  int wallDist = get_front_ir_dist(LEFT);
  float turnRate = 0;
  float multiplierL = 1.1 * (MIN_TURN_RADIUS * distToWall) / 43.5; //16.2; 
  float multiplierR = 30;


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
    //  printf("turn rate : %f, distToWall / thresholdDist : %f, distToWall %f\n", turnRate, wallDist / thresholdDist, distToWall);
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

void comeToStop() {
  while (get_front_ir_dist(RIGHT) > 15) {
    set_motors(15,15);
  }
  set_motors(0,0);
  get_motor_encoders(&leftEnc,&rightEnc);
  calcPosition(leftEnc,rightEnc);
  exit(0);
}