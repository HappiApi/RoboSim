#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "calcPos2.h"

#define followSpeed 35 //15
#define frontIROffset 4
#define followSide LEFT
#define thresholdDist 35.0
#define MIN_TURN_RADIUS 25
#define rightAngleTurnTarget 210
#define fastestNonSlipMotorValue 15
#define topTurnSpeed 40
#define LEFT 0
#define RIGHT 1

struct coordinates 
{
  double x;
  double y;
  // int lMSpeed;
  // int rMSpeed;
  struct coordinates * next;
};

typedef struct coordinates coords;

coords* head = NULL; coords* currentNode;

int leftEnc; int rightEnc; double xCurrent; double yCurrent;

void turn(float targetAngle, int direction) 
{
  int slowSpd = fastestNonSlipMotorValue;
  int fastSpd = topTurnSpeed - fastestNonSlipMotorValue;
  int currentSpd;
  double percentTurned = 0;
  int leftenc = 0;
  int rightenc = 0;
  int target = (double)(rightAngleTurnTarget * targetAngle) / 90;

  reset_motor_encoders();

  while(fabs(leftenc)<target && fabs(rightenc)<target) {
    get_motor_encoders(&leftenc,&rightenc);
    percentTurned = fabs(leftenc) / (double)target;
    if (percentTurned <= 0.5) {
      currentSpd = slowSpd + (int)(2 * percentTurned * (double)fastSpd);
    } else {
      currentSpd = slowSpd + (int)((1 - percentTurned) * (double)fastSpd);
    }
    if (direction == RIGHT) {
      set_motors(currentSpd, -currentSpd);
    } else {
      set_motors(-currentSpd, currentSpd);
    }
    // printf("%d %d\n",leftenc,rightenc);
    log_trail();
  }

  set_motors(0,0);
  usleep(40000);
}

void followBack() {
  float angleToNext;
  float xDist; float yDist;
  float distToNext;

  xDist = currentNode->x - currentNode->next->x;
  yDist = currentNode->y - currentNode->next->y;

  angleToNext = atan(xDist/yDist);
  printf("currentNode->x: %f, currentNode->next->x: %f, angle to next: %f\n",currentNode->x, currentNode->next->x, angleToNext * 180 / 3.141592);
  if (angleToNext>0) {
    turn(angleToNext, LEFT);
  } else {
    turn(-angleToNext, RIGHT);
  }

  distToNext = sqrt(xDist * xDist + yDist * yDist);

  get_motor_encoders(&leftEnc, &rightEnc);
  while (leftEnc < distToNext) {
    set_motors(15,15);
    get_motor_encoders(&leftEnc, &rightEnc);
  }
}


void comeToStop(coords * head, coords * currentNode) {
  while (get_front_ir_dist(RIGHT) > 15) {
    set_motors(15,15);
  }
  set_motors(0,0);
  get_motor_encoders(&leftEnc,&rightEnc);
  //calcPosition(leftEnc,rightEnc, &xCurrent, &yCurrent);
  calcPosition(leftEnc,rightEnc);
  currentNode = (coords *)malloc(sizeof(coords)); //creates new node
  currentNode->x = xCurrent; //assigns value of x and y coords for new node
  currentNode->y = yCurrent;
  currentNode->next = head;
  head = currentNode; //changes the head of the list to be the new node

  currentNode = head;
  turn(180, LEFT);
  int i = 1;
  while (currentNode) {
    printf("%i. x : %f, y : %f\n", i, currentNode->x, currentNode->y);
    followBack();
    currentNode = currentNode->next;
    i++;
  }
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

  usleep(10000);

  while (1) {
  	wallDist = get_front_ir_dist(LEFT);
    if (get_front_ir_dist(RIGHT) < 25) {
      comeToStop(head, currentNode);
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

    // printf("loop no: %i", i);

    //if (i % 2 == 0) {
      get_motor_encoders(&leftEnc,&rightEnc);
      calcPosition(leftEnc,rightEnc); 
      //calcPosition(leftEnc,rightEnc, &xCurrent, &yCurrent);
      //currentNode = (coords *)malloc(sizeof(coords)); //creates new node
      //currentNode->x = xCurrent; //assigns value of x and y coords for new node
      //currentNode->y = yCurrent;
      //currentNode->next = head;  //makes the new node point to the previous one
      //head = currentNode; //changes the head of the list to be the new node
      reset_motor_encoders();
   // }
   //i++;
  }

  return 1;
}