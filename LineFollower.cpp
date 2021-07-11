/*
 * LineFollower.cpp
 *
 *  Created on: Oct 15, 2020
 *      Author: gentov
 */

#include "LineFollower.h"
#include <math.h>

#define PI 3.14159265f

LineFollower::LineFollower(DrivingChassis* myChassis){
     this->robotChassis = myChassis;
}

void LineFollower::lineFollowBackwards(){
	  int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
	  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	  float leftCorrection = 0;
	  float rightCorrection = 0;
	  if(leftSensorValue >= ON_BLACK && rightSensorValue>= ON_BLACK)
	  {
	   if(canCountLine){
	      lineCount++;
	      // Mathematically speaking, this should only increment one of the following. Either
	      // row or column
	      float headingInRadians = (robotChassis->myChassisPose.heading)*(PI/180.0);
	      Serial.println(String(round(cos(headingInRadians))) + " " + String(round(sin(headingInRadians))));
	      robotChassis->myChassisPose.currentRow += round(cos(headingInRadians));
	      robotChassis->myChassisPose.currentColumn += round(sin(headingInRadians));
	      canCountLine = false;
	    }
	    //Serial.println("Line Count: " + String(lineCount));
	  }


	  else if(leftSensorValue >= ON_BLACK || rightSensorValue >= ON_BLACK){
			rightCorrection = (ON_BLACK - rightSensorValue)*lineFollowingKpBackwards;
			leftCorrection =  (leftSensorValue - ON_BLACK)*lineFollowingKpBackwards;
			canCountLine = true;
	  }
	  else{
		  canCountLine = true;
	  }
	  robotChassis->myleft -> setVelocityDegreesPerSecond(lineFollowingSpeedBackwards_mm_per_sec*MM_TO_WHEEL_DEGREES + leftCorrection);
      robotChassis->myright -> setVelocityDegreesPerSecond(-lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + rightCorrection);
}

void LineFollower::lineFollowForwards(){
	  int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
	  int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
	  float leftCorrection = 0;
	  float rightCorrection = 0;
	  if(leftSensorValue >= ON_BLACK && rightSensorValue>= ON_BLACK)
	  {
	   if(canCountLine){
	      lineCount++;
	      // Mathematically speaking, this should only increment one of the following. Either
	      // row or column. Since there are two markers for each row, we need to only count once every two markers.
	      float headingInRadians = (robotChassis->myChassisPose.heading)*(PI/180.0);
	      Serial.println(String(round(cos(headingInRadians))) + " " + String(round(sin(headingInRadians))));
	      robotChassis->myChassisPose.rowCount += abs(round(cos(headingInRadians)));
	      robotChassis->myChassisPose.colCount += round(sin(headingInRadians));
	      if(robotChassis->myChassisPose.rowCount == 2){
	          robotChassis->myChassisPose.currentRow += round(cos(headingInRadians));
	          robotChassis->myChassisPose.rowCount = 0;
	      }
	      robotChassis->myChassisPose.currentColumn = robotChassis->myChassisPose.colCount;
	      canCountLine = false; // This is meant as a line "debouncing". We don't want to catch the same line twice.
	    }
	    //Serial.println("Line Count: " + String(lineCount));
	  }


	  else if(leftSensorValue >= ON_BLACK || rightSensorValue >= ON_BLACK){
			rightCorrection = (ON_BLACK - rightSensorValue)*lineFollowingKpForwards;
			leftCorrection =  (leftSensorValue - ON_BLACK)*lineFollowingKpForwards;
			canCountLine = true;
	  }
	  else{
		  canCountLine = true;
	  }
	  robotChassis->myleft -> setVelocityDegreesPerSecond(-lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + leftCorrection);
      robotChassis->myright -> setVelocityDegreesPerSecond(lineFollowingSpeedForwards_mm_per_sec*MM_TO_WHEEL_DEGREES + rightCorrection);
}

void LineFollower::resetLineCount(){
    lineCount = 0;
}

