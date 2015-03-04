//
//  newtask3.c
//
//
//  Created by Boyang Pan on 26/02/2015.
//
//

#include <stdio.h>
#include "picomms.h"
#include <math.h>
#include <unistd.h>

#define CMPerTick (9.55*M_PI)/360 // cm per tick
#define roboWidth 23.5 // width of robot in cm

double unicornx = 0;
double unicorny = 0;

double totalangle = 0;
int previousrighttick = 0;
int previouslefttick = 0;
int currentlefttick, currentrighttick;

double leftencoderCM, rightencoderCM,currentangle,rl,rr,rm,deltax,deltay ;

void calculatePos(int leftencoder,int rightencoder)
{
    leftencoderCM = (double)leftencoder * CMPerTick ;
    rightencoderCM = (double)rightencoder * CMPerTick ;
    currentangle = (leftencoderCM - rightencoderCM)/ roboWidth ;
    
    if(currentangle == 0)
    {
        unicornx =  ((leftencoderCM + rightencoderCM) /2.0) * sin(totalangle) ;
        unicorny = (leftencoderCM + rightencoderCM) / 2.0 * cos(totalangle);
    }
    
    else
    {
        rl = leftencoderCM/currentangle;
        rr = rightencoderCM/currentangle;
        rm = (rl + rr) / 2.0 ;
        
        deltay = (rm * sin(totalangle + currentangle)) - (rm * sin(totalangle));
        deltax = (rm * cos(totalangle)) - (rm * cos(totalangle + currentangle));
        
        totalangle += currentangle ;
        unicornx += deltax ;
        unicorny += deltay ;
    }
    
    printf("X: %f, Y: %f\n", unicornx, unicorny);
    set_point((int) unicornx,(int) unicorny);
}

int main()
{
    connect_to_robot();
    initialize_robot();
    set_origin();
    set_ir_angle(-45,LEFT);
    set_motors(20,20);
    
    while(1)
    {
        
        if (get_front_ir_dist(LEFT) < 25)
        {
            set_motors(40,20);
        }
        if (get_front_ir_dist(LEFT) > 35)
        {
            set_motors(20,40);
        }
        
        get_motor_encoders(&currentlefttick, &currentrighttick);
        int movedlefttick = currentlefttick - previouslefttick ;
        int movedrighttick = currentrighttick - previousrighttick ;
        
        calculatePos(movedlefttick, movedrighttick);
        
        previouslefttick = currentlefttick ;
        previousrighttick = currentrighttick ;
    }
    
    return 1;
}