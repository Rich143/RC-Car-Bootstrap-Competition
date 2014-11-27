/* 
 * File:   main.c
 * Author: Richard Matthews
 *
 * Created on November 7, 2014, 11:32 PM
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


/**********************
 * PID Calculations   *
 **********************/

/*
 * The gains for P, I, and D
 */
const float Kp = 1;
const float Ki = 1;
const float Kd = 100;

const float dT = 0.000001; // The sampling rate of the pidCal

float integral;
float pidError;
float derivative;

// the output of the pidCalc
float output;

//The previous error
float preError;


//should slow down throttle for sharp turn
//PID control loop. Determines what to set as an output based on the error in the desired control variable

double pidCal(double setpoint, double actualPosition) {

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


struct Waypoint
{
	// Start Point
	long double startLat;
	long double startLon;

	// Current Position
	long double lat;
	long double lon;

	// End Point
	long double endLat;
	long double endLon;

	// The original distance to the waypoint
	long double endDistance;

	// Distance to waypoint
	long double distanceTraveled;

	// The current heading of the car
	int curHeading;

	// The desired heading
	int heading;

	// The value to set the steering to
	double turnAmount;
};


/*******************
 * GPS Functions   *
 *******************/


const long double TO_RAD = PI/180.0;
const long double R_EARTH = 6371; //KM

// Pass in a waypoint, returns the distance from the start position to the current position
long double distance(struct Waypoint *point) {
    long double dLat, dLon, sLat,sLon, eLat, eLon;
    sLat = point->startLat * TO_RAD;
    eLat = point->lat * TO_RAD;
    sLon = point->startLon * TO_RAD;
    eLon = point->lon * TO_RAD;
    dLat = eLat - sLat;
    dLon = eLon - sLon;

    long double a, sin_dLon, sin_dLat;
    sin_dLon = sin(dLon / 2);
    sin_dLat = sin(dLat / 2);
    
    a = sin_dLat*sin_dLat + cos(sLat)*cos(eLat)*sin_dLon*sin_dLon;

    long double c;
    c = 2 * atan2(sqrt(a), sqrt(1 - a));
    c *= -1000;

    long double d;
    d = R_EARTH * c;
    return d;
}


// Pass in a waypoint, and it computes the heading from the starting point to the current point
// should change to start to end
double heading(struct Waypoint * point)
{
	long double dLon, sLat,sLon, eLat, eLon;
    dLon = (point->endLon - point->startLon) * TO_RAD;
    sLat = point->startLat * TO_RAD;
    eLat = point->endLat * TO_RAD;
    sLon = point->startLon * TO_RAD;
    eLon = point->endLon * TO_RAD;

	double y = sin(dLon) * cos(eLat);
	double x = cos(sLat) * sin(eLat) - sin(sLat)*cos(eLat)*cos(dLon);

	double heading = atan2(y,x) * (180.0/PI);

	return heading;
}


void setStart(struct Waypoint * point)
{
	point->startLon = GPS.longitude;
	point->startLat = GPS.latitude;
}

void updatePosition (struct Waypoint * point)
{
	point->lon = GPS.longitude;
	point->lat = GPS.latitude;
}

void updateDistance(struct Waypoint * point)
{
	// Update the current position
	updatePosition(point);

	// set the new distance to the endpoint
	point->distanceTraveled = distance(point);
}


/*******************
 * Move Functions  *
 *******************/


void driveForward(float throttlePercent) {
    setThrottle(throttlePercent);
    setSteering(0);
}

void driveBackward(float throttlePercent) {
    setThrottle(-1 * throttlePercent);
    setSteering(0);
}

void turn(float throttlePercent, float steeringPercent) {
    setThrottle(throttlePercent);
    setSteering(steeringPercent);
}

void driveStop() {
    setThrottle(0);
}


/*****************
 *Time Functions *
 ****************/

unsigned long seconds() {
    return ((int) getSec() + (int) getMin() * 60);
}

void delay(int delayTime) {
    int waitTime = seconds() + delayTime;
    while (seconds() <= waitTime) {
        background();
        setDebugFloat((float)seconds());
    }
}



int main() {
    initTruck();
    while (true)
    {

        setThrottle(0); //Note that the first -20%/20% is a safety buffer region. Anything less than 20% is equivalent to no throttle.
        setSteering(0);

        // Set the first waypoint
        struct Waypoint point1; // First waypoint to drive to
        point1.endDistance = 5; // The distance to travel for phase 2

        // Wait for gps lock
        while(!isGPSLocked())
            background();

        // Set the startpoint
        setStart(& point1);

        driveForward(25);

        // Keep driving until we have reached the first endpoint
        while (fabsl(point1.distanceTraveled) < point1.endDistance)
        {
                updateDistance(& point1); // Find the new distance to the endpoint

                // Send the distance to help with debugging
                setDebugFloat((float)(point1.distanceTraveled));
                background();
        }

        // First endpoint reached, reset for the second
        driveStop();

        setStart(& point1);

        driveBackward(25);

        // Keep driving backward until we have reached the second enpoint
        while (fabsl(point1.distanceTraveled) < point1.endDistance)
        {
                updateDistance(& point1);

                // Send the distance to help with debugging
                setDebugFloat((float)(point1.distanceTraveled));
                background();
        }

        driveStop();

        while(true);
    }

	return 0;
}

