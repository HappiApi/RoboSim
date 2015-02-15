#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
#include "calcPos.h"

#define followSpeed 40 //15
#define frontIROffset 4
#define followSide LEFT
#define thresholdDist 35.0
#define TRACK 23.5
#define MIN_TURN_RADIUS 25

struct coordinates 
{
  double x;
  double y;
  struct coordinates * next;
};

typedef struct coordinates coords;

coords* head = NULL; coords* currentNode;

int leftEnc; int rightEnc; double xCurrent; double yCurrent;

void comeToStop(coords * head, coords * currentNode) {
  while (get_front_ir_dist(RIGHT) > 15) {
    set_motors(15,15);
  }
  set_motors(0,0);
  get_motor_encoders(&leftEnc,&rightEnc);
  calcPosition(leftEnc,rightEnc, &xCurrent, &yCurrent);
  currentNode = (coords *)malloc(sizeof(coords)); //creates new node
  currentNode->x = xCurrent; //assigns value of x and y coords for new node
  currentNode->y = yCurrent;
  currentNode->next = head;
  head = currentNode; //changes the head of the list to be the new node

  currentNode = head;
  int i = 1;
  while (currentNode) {
    printf("%i. x : %f, y : %f\n", i, currentNode->x, currentNode->y);
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
  coords * head; 
  coords * currentNode; 
  head = NULL;

  int wallDist = get_front_ir_dist(LEFT);
  float turnRate = 0;
  float multiplierL = 0.5*followSpeed;
  float multiplierR = followSpeed;


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

    if (i % 8 == 0) {
      get_motor_encoders(&leftEnc,&rightEnc);
      calcPosition(leftEnc,rightEnc, &xCurrent, &yCurrent);
      currentNode = (coords *)malloc(sizeof(coords)); //creates new node
      currentNode->x = xCurrent; //assigns value of x and y coords for new node
      currentNode->y = yCurrent;
      currentNode->next = head;  //makes the new node point to the previous one
      head = currentNode; //changes the head of the list to be the new node
      reset_motor_encoders();
    }
    i++;
    usleep(10000);
  }

  return 1;
}