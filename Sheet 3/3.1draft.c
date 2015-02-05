// Pair22 Blackwell, Hasthanasombat

/* I have got the angle calculator working, there is some skeleton code for working out the coordinates, but needs fixing*/

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "picomms.h"

#define distanceToWall 20
#define distanceToFront 30
#define endOfWallMultiplier 2
#define slowSpeed 15
#define fastSpeed 30
#define superSpeed 50
#define frontIROffset 4
#define followSide LEFT
#define ROBOT_TRACK 236 //mm
#define EncToDistance 1000 / 954

int straight = 1;
int turn = 0;
double currentAngle = 0.0;
double totalDist = 0;

struct coordinates {
  double x;
  double y;
};

struct coordinates coords[300]; 
int coordNo = 1;

double angleTurned () {
  int lDist; int rDist;
  get_motor_encoders(&lDist, &rDist);
  double angle;

  angle = (lDist - rDist) / (double)ROBOT_TRACK;  //returns the angle in radians, +ve for right turn, -ve for left
  currentAngle += angle;
  printf("\n\n%f, currentAngle: %f\n\n", angle * 180 / 3.1415, currentAngle* 180 /3.141); // Some Error

  reset_motor_encoders();

  return angle;
}

// void turnDistTravelled () {
//   int lDist; int rDist;
//   get_motor_encoders(&lDist, &rDist);
//   lDist = lDist * 1000 / 954; rDist = rDist * 1000 / 954;
//   double dist = 0;
//   double turnAngle = angleTurned();
//   double centreRad = (lDist + rDist) / 2 / turnAngle;  //radius of the circle to the centre of the robot, this should account for negative distances travelled also
//   double outerRad = fmax(rDist, lDist) / turnAngle;

//   dist = centreRad * turnAngle;

//   coords[coordNo].x = coords[coordNo - 1].x + outerRad - (centreRad * cos(currentAngle + turnAngle));
//   coords[coordNo].y = coords[coordNo - 1].y + outerRad - (centreRad * sin(currentAngle + turnAngle));
//   printf("%f, %f\n", coords[coordNo].x, coords[coordNo].y);
//   coordNo++;

//   printf("%f", currentAngle);

//   reset_motor_encoders();

//   totalDist += dist; 
// }

void addStraightDist () {
  int lDist; int rDist;
  get_motor_encoders(&lDist, &rDist);
  lDist = lDist * EncToDistance; rDist = rDist * EncToDistance;

  coords[coordNo].x = coords[coordNo - 1].x + lDist * sin(currentAngle);
  coords[coordNo].y = coords[coordNo - 1].y + rDist * cos(currentAngle);
  printf("%f, %f\n", coords[coordNo].x, coords[coordNo].y);
  coordNo++;

  totalDist += (lDist + rDist) / 2;
  reset_motor_encoders();
}

void setSensors()
{
  set_ir_angle(LEFT, -45);
  set_ir_angle(RIGHT,-75);
}

void adjustParallel()
{
  if (get_side_ir_dist(followSide) > get_front_ir_dist(followSide)-frontIROffset)   
  {
    turn = 1;
    angleTurned();
    set_motors(fastSpeed,slowSpeed);
//    printf("RIGHT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
  }

  if(get_side_ir_dist(followSide) < get_front_ir_dist(followSide)-frontIROffset)
  {
    turn = 1;
    angleTurned();
    set_motors(slowSpeed,fastSpeed);
//    printf("LEFT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
  } 
}

void lookForFront()
{
  if(get_front_ir_dist(RIGHT) < distanceToFront)
  {
      if (straight == 1) {
        straight = 0;
        addStraightDist();
        turn = 1;
      } else {
        angleTurned();
      //  turnDistTravelled();
      }
    while(get_front_ir_dist(RIGHT) < distanceToFront)
    {      
      set_motors(fastSpeed,-fastSpeed);
//      printf(" FRONT FR%d\n",get_front_ir_dist(RIGHT) );
    }
  }
}

void lookForEnd()
{
  if( endOfWallMultiplier * get_side_ir_dist(followSide) < get_front_ir_dist(followSide)-frontIROffset)
  {
      if (straight == 1) {
        straight = 0;
        addStraightDist();
        turn = 1;
      } else {
        angleTurned();
       // turnDistTravelled();
      }
      
    while(endOfWallMultiplier * get_side_ir_dist(followSide) < get_front_ir_dist(followSide))
    {
      set_motors(slowSpeed,superSpeed);
//      printf("UTURN SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide) );
    }
  }
}

void goStraight()
{
  if( get_side_ir_dist(followSide) == get_front_ir_dist(followSide)-frontIROffset)
  {
    if (turn == 1) {
      turn = 0;
      angleTurned();
     // turnDistTravelled();
      straight = 1;
    }
    
    set_motors(fastSpeed,fastSpeed);
//    printf("STRAIGHT SL%d FL%d\n", get_side_ir_dist(followSide), get_front_ir_dist(followSide));
  }
}

void setCoords()
{
  coords[0].x = 0.0; 
  coords[0].y = 0.0;
}

int main()
{
  connect_to_robot();
  initialize_robot();
  setSensors();
  setCoords();

  while(1)
  {
    adjustParallel();
    lookForFront();
    lookForEnd();
    goStraight();
    
  }
}