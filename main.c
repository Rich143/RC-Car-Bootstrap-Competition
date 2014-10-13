/* 
 * File:   main.c
 * Author: Chris Hajduk
 *
 * Created on August 21, 2014, 11:32 PM
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "GPS.h"
#include "main.h"
#include "HunterTruckAPI.h"
#include "debug.h"

#include "OutputCompare.h"
#include "InterchipDMA.h"
#include "PWM.h"

_FOSCSEL(FNOSC_FRC); // Internal FRC oscillator
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_NONE);
_FWDT(FWDTEN_OFF & WDTPOST_PS2048 & WDTPRE_PR128); //32,128
/*
 * 
 */
/**********************
 * PID Calculations   *
 **********************/

/*
 * The gains for P, I, and D
 */
const float Kp = 1;
const float Ki = 1;
const float Kd = 1;

const float dT = 0.01; // The sampling rate of the pidCal

float integral;
float pidError;
float derivative;

float output;

//The previous error
float preError;

//should slow down throttle for sharp turn
//PID control loop. Determines what to set as an output based on the error in the desired control variable

float pidCal(float setpoint, float actualPosition) {
    // calculate the difference between
    // the desired value and the actual value
    pidError = setpoint - actualPosition;

    // track error over time, scaled to the timer interval
    integral += (pidError * dT);

    // determine the amount of change from the last time checked
    derivative = (pidError - preError) / dT;

    // calculate how much to drive the output in order to get to the
    // desired setpoint.
    output = (Kp * pidError) + (Ki * integral) + (Kd * derivative);

    if (output)
        // remember the error for the next time around.
        preError = pidError;

    return output;
}

/*
 * Prototypes
 */
float map(float, float, float, float, float);
float distance(float, float, float, float);
void driveForward(float, float);
void driveBackward(float, float);
void turnRight(float, float, float);
void driveStop();
void startTimer(Timer, float);
boolean isExpiredTimer(Timer);
unsigned long seconds();
void updateSeconds();
void delay(int);

//Maps a number x in input range iStart to iEnd to range oStart to oEnd

float map(float x, float iStart, float iEnd, float oStart, float oEnd) {
    double slope = 1.0 * (oEnd - oStart) / (iEnd - iStart); //calculate the scaling factor
    return (oStart + slope * (x - iStart));
}

/*
 * GPS Functions
 */
#define TO_RAD (PI/180)
#define R_EARTH 6371 //KM
#define STEERING_GAIN

float distance(float * start, float * end) {
    float dlat, dlong;
    dlat = (start[0] - end[0]) * TO_RAD;
    dlong = (start[1] - end[1]) * TO_RAD;
    start[0] *= TO_RAD;
    end[0] *= TO_RAD;
    start[1] *= TO_RAD;
    end[1] *= TO_RAD;

    float a;
    a = sin(dlat / 2) * sin(dlat / 2) + cos(start[0]) * cos(end[0])
            * sin(dlong / 2) * sin(dlong / 2);
    float c;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    c *= -1000;
    float d;
    d = R_EARTH * c;
    return d;
}
/*
 * Move Functions
 */

/*
void correctHeading(float heading)
{
    int currentHeading = getHeading(); //find out the current heading
    int headingDif = heading - currentHeading;
    if (abs(headingDif) > 180)  // The max heading difference is 180 since heading is a cirlce
    {
        if (headingDif >=0)
            headingDif -= 360; // Correct heading diff since its shorter to turn the other way
        else
            headingDif += 360; // Correct heading diff since its shorter to turn the other way
    }
    int steeringCorrection = map(heading - currentHeading, -180, 180, -100, 100) * STEERING_GAIN;
}
 */

void driveForward(float throttlePercent) {
    setThrottle(throttlePercent);
    setSteering(0);
    // startTimer(drivingTimer, time);
}

void driveBackward(float throttlePercent) {
    setThrottle(-1 * throttlePercent);
    setSteering(0);
    // startTimer(drivingTimer, time);
}

void turn(float throttlePercent, float steeringPercent) {
    setThrottle(throttlePercent);
    setSteering(steeringPercent);
    // startTimer(drivingTimer, time);
}

void driveStop() {
    setThrottle(0);
}


/*******
 * Timers
 ********/
double expiryTime[NUMBER_OF_TIMERS];
boolean expiredTimers[NUMBER_OF_TIMERS];

// Start a timer with that runs for lenght seconds

void startTimer(Timer timer, float length) {
    expiryTime[timer] = length + seconds();
}

// returns true if the timer is expired

boolean isExpiredTimer(Timer timer) {
    return (seconds() >= expiryTime[timer]);
}

/*
Timer areTimersExpired()
{
    for (int i = 0; i <= NUMBER_OF_TIMERS; i++)
    {
        if(isExpiredTimer(Timer(i)))
        {
            expiredTimers[i] = true;
        }
    }
}
 */

void delay(int delayTime) {
    int waitTime = seconds() + delayTime;
    while (seconds() <= waitTime) {
        background();
        setDebugFloat(seconds);
    }
}
/*********
 *Time Functions
 *
 **********/


//unsigned long seconds = 0;

unsigned long seconds() {
    return ((int) getSec() + (int) getMin() * 60);


}

/*
void updateSeconds()
{
    seconds = seconds();
}
 */




MoveState moveState = waitingToStart;

Task task = phase2;

int main() {
    initTruck();
    while (true) {
        //This is how you move the car. Throttle goes from -100% to 100%. Steering goes from -100 to 100%.
        setThrottle(0); //Note that the first -20%/20% is a safety buffer region. Anything less than 20% is equivalent to no throttle.
        setSteering(0);

        long double pos[2]; // the current position, must update this manually by calling getPos(pos)
        long double startPos[2]; // the position the car refers to as its starting position when doing distance calculation, must be set
        float distTravelled; // the distance the car has travelled calculated by distance()

        moveState = waitingToStart; // the current move state
        setDebugChar('w');
        switch (task) {
            case(phase1):
                driveForward(30);
                delay(10);

                driveStop();
                delay(3);

                driveBackward(30);
                delay(10);

                driveStop();
                delay(3);

                turn(30, 40);
                delay(10);

                driveStop();
                delay(3);

                turn(30, -40);
                delay(10);
                break;

            case(phase2):
                boolean Done = false;
                float distanceToTravel = 3; //Distance to travel in phase 2 (meters)
                setDebugChar('d');
                delay(2); //delay on startup
                while (!Done) {
                    switch (moveState) {
                        case(waitingToStart):
                            setDebugChar('G'); // Print to the debuger to notify waiting for GPS lock
                            setDebugInt((int) isGPSLocked()); // Print to debugger whether we have gps lock
                            if (isGPSLocked()) // Wait untill we have gps lock before starting
                            {
                                delay(1); // Delay to allow the new debug to print

                                setDebugChar('F'); // debug print that we are driving foward
                                moveState = drivingForward;

                                getPosition(pos); // update our position
                                startPos[0] = pos[0]; // Set the start position, held in a long double array of 2
                                startPos[1] = pos[1];

                                driveForward(25);
                            }
                            break;

                        case(drivingForward):
                            getPosition(pos); // update position
                            distTravelled = distance(startPos, pos); // compute the distance travelled
                            setDebugFloat(distTravelled); // Print the distance travelled
                            if (distTravelled >= distanceToTravel) {
                                driveStop();
                                setDebugChar('s');
                                delay(1);
                                moveState = drivingBackward;
                                setDebugChar('b');

                                getPosition(pos); // update position
                                startPos[0] = pos[0]; // set start position
                                startPos[1] = pos[1];

                                driveBackward(25);
                            }
                            break;

                        case(drivingBackward):
                            getPosition(pos);
                            distTravelled = distance(startPos[0], startPos[1], pos[0], pos[1]);
                            setDebugFloat(distTravelled);
                            if (distTravelled >= distanceToTravel) {
                                Done = true;
                            }

                    }

                    background();
                }
                break;

            case (phase3):
                setDebugChar('d');
                delay(2); //delay on startup
                Done = false;
                moveState = waitingToStart;

                int heading; // The current heading
                int endHeading; // The desired heading after the turn

                float turnAmount; // What to set steering to based of pidCal

                long nextPidCal = seconds(); // The next time to perform the pid calculation
                while (!Done) {
                    switch (moveState) {
                        case(waitingToStart):
                            setDebugChar('g');
                            if (isGPSLocked()) {
                                setDebugChar('f');
                                moveState = drivingForward;
                                driveForward(50);

                                getPosition(pos);
                                startPos[0] = pos[0];
                                startPos[1] = pos[1];
                            }
                            break;

                        case(drivingForward):
                            getPosition(pos); // update position
                            distTravelled = distance(startPos, pos); // compute the distance travelled
                            setDebugFloat(distTravelled); // Print the distance travelled
                            if (distTravelled >= 3) { // Drive Forward 3 meters
                                driveStop();
                                setDebugChar('s');
                                delay(1); // Delay to allow debugs to print
                                moveState = turningRight;
                                setDebugChar('r');
                                endHeading = getHeading() + 90; // Set the heading after the 90? right turn
                            }
                            break;

                        case(turningRight):
                            heading = getHeading();
                            driveForward(30);

                            turnAmount = pidCal(endHeading, heading); // Determine what to steer the car
                            if (turnAmount > 100)
                                turnAmount = 100;
                            else if (turnAmount < -100)
                                turnAmount = -100;
                            setSteering(turnAmount); // Set the correct Steerign

                            setDebugFloat(turnAmount); // Print the turnAmount
                            setDebugInt(heading); // Print the current heading

                            if (heading >= endHeading) {
                                setSteering(0); // Stop turning
                                driveStop();
                                setDebugChar('s');
                                delay(1);
                                moveState = turningLeft;
                                setDebugChar('l');
                                endHeading = getHeading() - 90;
                            }

                        case(turningLeft):
                            heading = getHeading();
                            driveForward(30);

                            turnAmount = pidCal(endHeading, heading); // Determine what to steer the car
                            if (turnAmount > 100)
                                turnAmount = 100;
                            else if (turnAmount < -100)
                                turnAmount = -100;
                            setSteering(turnAmount); // Set the correct Steerign

                            setDebugFloat(turnAmount); // Print the turnAmount
                            setDebugInt(heading); // Print the current heading

                            if (heading <= endHeading) {
                                setSteering(0); // Stop turning
                                driveStop();
                                setDebugChar('s');
                                delay(1);
                                moveState = turningLeft;
                                setDebugChar('l');
                                endHeading = getHeading() - 90;
                            }



                    }

                }
        }

        driveStop();

        while (true);

        //        This is an example of how you can print the GPS time to the debugging interface.
        //        char str[16];
        //        sprintf((char *)&str, "GPS: %f", GPS.time);
        //        debug((char *)&str);

        // updateSeconds();
        background();
    }
    return (EXIT_SUCCESS);
}
