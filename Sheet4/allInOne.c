#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <time.h>
#include "picomms.h"
//#include "calcPos.h"

#define FOLLOW_SPEED 35 //15
#define FOLLOW_BK_SPD 15
#define THRESHOLD_DIST 35.0
#define MIN_TURN_RADIUS 25
#define RIGHT_ANGLE_TICKS 210
#define NO_SLIP_TURN_SPD 15
#define TOP_TURN_SPEED 40
#define LEFT 0
#define RIGHT 1
#define ENC_TO_CM (9.55 * 3.141592 / 360)
#define ROBOT_WIDTH 22.5   // In cm

struct coordinates 
{
  double x;
  double y;
  int position;
  struct coordinates * next;
};

typedef struct coordinates coords;

coords* head = NULL; coords* currentNode; int nodePosition = 1;

int i = 0; double xCurrent, yCurrent;
double prevAngle = 0;   // initial condition previous angle = 0
double xCoord = 0; double yCoord = 0;

/*finds the angle between the heading direction and a point*/
double angleToTargetPoint(coords * target) {
  double angleFromY, deltaX, deltaY, angleFromCurrent;// diagDist, aheadDist, acrossDist;

  deltaX = target->x - xCurrent;
  deltaY = target->y - yCurrent;

  angleFromY = fabs(atan(deltaX / deltaY));
  if (deltaX > 0) {
    angleFromY = M_PI - angleFromY; 
  } else {
    angleFromY = M_PI + angleFromY;
  } 
  angleFromCurrent = angleFromY - prevAngle;

  printf("\n\ndelta x : %f, delta y : %f, angleFromY : %f, angleFromCurrent : %f",deltaX, deltaY, angleFromY * 180 / M_PI, angleFromCurrent * 180 / M_PI);

  return angleFromCurrent * 180 / M_PI;
}

/*finds the distance between the robot and a point*/
double calcDistToPoint (coords * temp) {
  double deltaX, deltaY, distToPoint;

  deltaX = temp->x - xCurrent;
  deltaY = temp->y - yCurrent; 
  distToPoint = sqrt(deltaX * deltaX + deltaY * deltaY);

  return distToPoint;
}

/*finds the furthest coordinate ahead within a set radius (target) */
coords * findTarget () {
  coords * temp = currentNode;
  double target = 20;
  int removedBehind = 0;
  double distToNode, closestNodeDist = 30.0;
  
    while(1) {
      distToNode = calcDistToPoint(temp);
      if (distToNode < closestNodeDist) {
        currentNode = temp;
        closestNodeDist = distToNode;
      }
      if (temp->position == 1) {  //don't allow it to try searching beyond the end of the list
        return temp;
      }
      if (distToNode < target && calcDistToPoint(temp->next) > target && temp->y < yCurrent) {  //if this is too cpu heavy for the RasPi then change calcDistToPoint(temp->next) to calcDistToPoint(temp->next->next->next) and adjust 3 lines below as noted 
        return temp;
      } else {
        temp = temp->next;  //change this to temp->next->next->next as well!
      }
    }
}

/*Function that handles the following back of a linked list of coordinates*/
void followBack() {
  coords * target;
  int i;
  double angleToTarget;
  double ratio = 0.03;
  while(1) {

    target = findTarget();
    if (currentNode-> position < 3) {
      set_motors(0,0);
      exit(0);
    }

    printf("\n\nfollow back, position %i, target pos: %i", currentNode->position, target->position);
    
    angleToTarget = angleToTargetPoint(target);

    printf("\n\n%f", angleToTarget);
    if (angleToTarget > 0) {
      set_motors(FOLLOW_BK_SPD, (int)((1 - ratio * angleToTarget) * (float)FOLLOW_BK_SPD));
    } else {
      set_motors((int)((1.0 - (ratio * -angleToTarget)) * FOLLOW_BK_SPD), FOLLOW_BK_SPD);
    }

    calcPosition();

    usleep(10000);

  }
}

void calcPositionMaths(int leftEnc, int rightEnc) 
{
  double angleTurned;   // current turning angle
  double deltaX, deltaY;
  double Rl, Rr, Rm;
  double p, q, a, b ;   // Should probably find better name


  if (fabs(leftEnc - rightEnc) < 1)
  {
    xCoord += (double)leftEnc * ENC_TO_CM * sin(prevAngle) ; 
    yCoord += (double)rightEnc * ENC_TO_CM * cos(prevAngle) ;
  }
  else
  {
    angleTurned = (double)( leftEnc - rightEnc ) * ENC_TO_CM / ROBOT_WIDTH ;
    Rl = (double)leftEnc / angleTurned ;
    Rr = (double)rightEnc / angleTurned ;
    Rm = ( Rl + Rr ) / 2.0 ;
    
    p = Rm * sin(prevAngle + angleTurned) ;
    q = Rm * sin(prevAngle) ;
    deltaY = p - q ;

    a = Rm * cos(prevAngle) ;
    b = Rm * cos(prevAngle + angleTurned) ;
    deltaX = a - b ;

    xCoord += deltaX * ENC_TO_CM ;
    yCoord += deltaY * ENC_TO_CM ;
    if (prevAngle + angleTurned < 0) {
      prevAngle = 2 * M_PI + prevAngle + angleTurned;
    } else if (prevAngle + angleTurned > 2 * M_PI) {
      prevAngle = prevAngle + angleTurned - 2 * M_PI;
    } else {
      prevAngle += angleTurned ;  //adjusted to += 
    }
  }
  // log_trail();
  xCurrent = xCoord;
  yCurrent = yCoord;
  set_point(round(xCurrent), round(yCurrent)); //set point in centimetres
  printf("X : %f Y : %f Angle : %f\n", xCurrent, yCurrent, prevAngle * 180 / 3.141592);
  printf("Dist Travelled: %imm, angle travelled at: %f degrees\n", (int)sqrt((xCurrent * xCurrent) + (yCurrent * yCurrent)), atan(xCurrent / yCurrent) * 180 / 3.141592); 
}

/*apply data smoothing to the encoder readings within this function*/
void calcPosition() {
  static int lEnc[3], rEnc[3], lEncFinal, rEncFinal; static int leftEncPrev = 0; static int rightEncPrev = 0;
  if (i % 3 == 0) {
    get_motor_encoders(&lEnc[0], &rEnc[0]);
  } else if (i % 3 == 1) {
    get_motor_encoders(&lEnc[1], &rEnc[1]);
  } else {
    get_motor_encoders(&lEnc[2], &rEnc[2]);
  }
  if (i > 1) {
    lEncFinal = round((float)(lEnc[0] + lEnc[1] + lEnc[2]) / 3.0);
    rEncFinal = round((float)(rEnc[0] + rEnc[1] + rEnc[2]) / 3.0);
    calcPositionMaths(lEncFinal - leftEncPrev, rEncFinal - rightEncPrev);
    leftEncPrev = lEncFinal;
    rightEncPrev = rEncFinal;
  } 
  i++;
}

void turn(float targetAngle, int direction) 
{
  int slowSpd = NO_SLIP_TURN_SPD;
  int fastSpd = TOP_TURN_SPEED - NO_SLIP_TURN_SPD;
  int currentSpd;
  double percentTurned = 0;
  int leftenc, lEncInitial;
  int rightenc, rEncInitial;
  int target = (double)(RIGHT_ANGLE_TICKS * targetAngle) / 90;

  get_motor_encoders(&lEncInitial, &rEncInitial);
  rightenc = rEncInitial; leftenc = lEncInitial;

  while(fabs(lEncInitial - leftenc)<target && fabs(rEncInitial - rightenc)<target) {
    get_motor_encoders(&leftenc,&rightenc);
    percentTurned = fabs(leftenc - lEncInitial) / (double)target;
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
  }

  set_motors(0,0);
  usleep(40000);
}

void comeToStop() {
  while (get_front_ir_dist(RIGHT) > 15) {
    set_motors(15,15);
  }
  set_motors(0,0);
  calcPosition();
  currentNode = (coords *)malloc(sizeof(coords)); //creates new node
  currentNode->x = xCurrent; //assigns value of x and y coords for new node
  currentNode->y = yCurrent;
  currentNode->position = nodePosition;
  currentNode->next = head;
  head = currentNode; //changes the head of the list to be the new node
  nodePosition++;

  currentNode = head;
  turn(180, LEFT);
  calcPosition();
  followBack();
}

int main() {
	connect_to_robot();
	initialize_robot();
  set_origin();
  set_ir_angle(RIGHT, -45);

  int wallDist;
  float turnRate = 0;
  float multiplierL = 0.5 * FOLLOW_SPEED;
  float multiplierR = FOLLOW_SPEED;

  usleep(10000);

  while (1) {
  	wallDist = get_front_ir_dist(LEFT);
    if (get_front_ir_dist(RIGHT) < 25) {
      comeToStop();
    }
  	if (wallDist == THRESHOLD_DIST) {
  		set_motors(FOLLOW_SPEED, FOLLOW_SPEED);
  	} else if (wallDist < THRESHOLD_DIST) {
  		turnRate = 1.0 - (0.8 * wallDist / THRESHOLD_DIST);
  		set_motors(FOLLOW_SPEED, (int)(FOLLOW_SPEED - multiplierR * turnRate));  //Turn right
  	} else {
      if (wallDist > 70) {
        turnRate = 1.0;
      } else {
  	   	turnRate = 1.5 * (wallDist / THRESHOLD_DIST - 1.0);
  		}
      set_motors((int)(FOLLOW_SPEED - multiplierL * turnRate), FOLLOW_SPEED);
  	}
      calcPosition();
      currentNode = (coords *)malloc(sizeof(coords)); //creates new node
      currentNode->x = xCurrent; //assigns value of x and y coords for new node
      currentNode->y = yCurrent;
      currentNode->position = nodePosition;
      currentNode->next = head;  //makes the new node point to the previous one
      head = currentNode; //changes the head of the list to be the new node
      nodePosition++;
  }

}

/*implement a data smoothing for the read motor encoder values*/