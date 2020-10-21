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
	  float leftCorrection = 0;
	  float rightCorrection = 0;

	  if(leftSensorValue >= ON_BLACK && rightSensorValue>= ON_BLACK)
	  {
	    // reset the timeout timer.
	    if(canCountLine){
	      lineCount++;
	      canCountLine = false;
	    }
	    //Serial.println("Line Count: " + String(lineCount));
	  }


	  else if(leftSensorValue >= ON_BLACK || rightSensorValue >= ON_BLACK){
			rightCorrection = (ON_BLACK - rightSensorValue)*lineFollowingKp;
			leftCorrection =  (leftSensorValue - ON_BLACK)*lineFollowingKp;
	  }

//	  else if(leftSensorValue < ON_BLACK && rightSensorValue >= ON_BLACK){
//			rightCorrection = -(rightSensorValue - ON_BLACK)*lineFollowingKp;
//			leftCorrection =  -(ON_BLACK - leftSensorValue)*lineFollowingKp;
//	  }

//	  else if(leftSensorValue < ON_BLACK && rightSensorValue >= ON_BLACK){
//	    // turn right
////		rightCorrection = (ON_BLACK - rightCorrection)*.05;
////		leftCorrection = (leftSensorValue - ON_BLACK)*.05;
//	    rightCorrection = -50;
//	    leftCorrection = -50;
//	    canCountLine = true;
//	  }
////
//	  else if(leftSensorValue >= ON_BLACK && rightSensorValue < ON_BLACK){
//	    // turn left
//	    //Serial.println("Turning Left");
//	    rightCorrection = 50;
//	    leftCorrection = 50;
////		rightCorrection = (rightSensorValue - ON_BLACK)*.05;
////		leftCorrection = (ON_BLACK - leftSensorValue)*.05;
//	    canCountLine = true;
//	  }
//
//	  else{
//	    //Serial.println("Straddling Line");
//	    canCountLine = true;
//	  }
	  //Serial.println("giving vel command");
	  // Only works backwards
	  robotChassis->myleft -> setVelocityDegreesPerSecond(lineFollowingSpeed_mm_per_sec*MM_TO_WHEEL_DEGREES + leftCorrection);
      robotChassis->myright -> setVelocityDegreesPerSecond(-lineFollowingSpeed_mm_per_sec*MM_TO_WHEEL_DEGREES + rightCorrection);
	  // if not timeout
	    // set velocity
}
void LineFollower::resetLineCount(){
	lineCount = 0;
}

