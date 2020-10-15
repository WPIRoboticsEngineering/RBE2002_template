/*
 * LineFollower.cpp
 *
 *  Created on: Oct 15, 2020
 *      Author: gentov
 */

#include "LineFollower.h"

LineFollower::LineFollower(DrivingChassis* myChassis){
     this->robotChassis = myChassis;
}

void LineFollower::lineFollow(){
	// line following. if both sensors are on white full steam ahead but timeout eventually.
	  // we need the timeout cause otherwise
	  int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
	  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	  int leftCorrection = 0;
	  int rightCorrection = 0;

	  if(leftSensorValue >= ON_BLACK && rightSensorValue>= ON_BLACK)
	  {
	    // reset the timeout timer.
	    if(canCountLine){
	      lineCount++;
	      canCountLine = false;
	    }
	    Serial.println("Line Count: " + String(lineCount));
	  }

	  else if(leftSensorValue < ON_BLACK && rightSensorValue >= ON_BLACK){
	    // turn right
	    //Serial.println("Turning Right");
	    leftCorrection = 50;
	    rightCorrection = -50;
	    canCountLine = true;
	  }

	  else if(leftSensorValue >= ON_BLACK && rightSensorValue < ON_BLACK){
	    // turn left
	    //Serial.println("Turning Left");
	    rightCorrection = 50;
	    leftCorrection = -50;
	    canCountLine = true;
	  }

	  else{
	    //Serial.println("Straddling Line");
	    canCountLine = true;
	  }
	  //Serial.println("giving vel command");
	  // Only works backwards
	  robotChassis->myleft -> setVelocityDegreesPerSecond(100*MM_TO_WHEEL_DEGREES + rightCorrection);
      robotChassis->myright -> setVelocityDegreesPerSecond(-100*MM_TO_WHEEL_DEGREES - leftCorrection);
	  //robotChassis->myright -> setVelocityDegreesPerSecond(100*MM_TO_WHEEL_DEGREES + leftCorrection);
	  //robotChassis->myleft -> setVelocityDegreesPerSecond(-100*MM_TO_WHEEL_DEGREES - rightCorrection);
	  // if not timeout
	    // set velocity
}

void LineFollower::resetLineCount(){
	lineCount = 0;
}

