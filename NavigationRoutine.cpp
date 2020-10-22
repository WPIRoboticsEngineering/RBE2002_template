/*
 * NavigationRoutine.cpp
 *
 *  Created on: Oct 22, 2020
 *      Author: gentov
 */
#include "NavigationRoutine.h"

bool navigate(int row, int col, DrivingChassis* drivingChassis, LineFollower* lineSensor){
	switch(navState){
		case INITIALIZE_NAVIGATION:
			Serial.println("INIT NAV");
			// if we're in the outerlane, then there really isn't a need to find the outerlane
			if(drivingChassis->myChassisPose.currentColumn == 0){
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;
			}
			break;
		case TURN_TOWARDS_CORRECT_COLUMN:
			Serial.println("TURNING TOWARDS COLUMN");
			// determine what is our first state
			if(drivingChassis->myChassisPose.currentRow != row){
				// if our current row isn't our goal row, then we need to navigate to the outer edge
				// first
				/// TODO: check where we are, maybe our orientation is correct
				if(drivingChassis->turnToHeading(90, 2500) == REACHED_SETPOINT){
					 navState = FINDING_OUTER_EDGE;
				}
			}
			else{
				// otherwise, we just need to find the right column
				if(drivingChassis->myChassisPose.currentColumn > col){
					if(drivingChassis->turnToHeading(-90, 2500) == REACHED_SETPOINT){
						navState = FINDING_COLUMN;
					}
				}
				else{
					if(drivingChassis->turnToHeading(90, 2500) == REACHED_SETPOINT){
						navState = FINDING_COLUMN;
					}
				}
			}
			break;
		case FINDING_OUTER_EDGE:
			Serial.println("FINDING COL: 0, CURRENT COL: " + String(drivingChassis->myChassisPose.currentColumn));
			// navigate until column == 0
			if(drivingChassis->myChassisPose.currentColumn != 0){
				// if the column is wrong.
				lineSensor->lineFollowForwards();
			}
			else{
				drivingChassis->stop();
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			break;
		case FINDING_ROW:
			Serial.println("FINDING ROW: " + String(row) +  "CURRENT ROW: " + String(drivingChassis->myChassisPose.currentRow));
			if(drivingChassis->myChassisPose.currentRow != row){
				// if the row is wrong.
				lineSensor->lineFollowForwards();
			}
			else{
				drivingChassis->stop();
				if(col != 0){
					navState = TURN_TOWARDS_CORRECT_COLUMN;
				}
			}
			break;
		case TURN_TOWARDS_CORRECT_ROW:
			Serial.println("TURNING TOWARDS ROW");
			// otherwise, we just need to find the right column
			if(drivingChassis->myChassisPose.currentRow > row){
				if(drivingChassis->turnToHeading(180, 2500) == REACHED_SETPOINT){
					navState = FINDING_ROW;
				}
			}
			else{
				if(drivingChassis->turnToHeading(0, 2500) == REACHED_SETPOINT){
					navState = FINDING_ROW;
				}
			}
			break;
		case FINDING_COLUMN:
			Serial.println("FINDING COL: " + String(col) +  "CURRENT COL: " + String(drivingChassis->myChassisPose.currentColumn));
			if(drivingChassis->myChassisPose.currentColumn != col){
				// if the row is wrong.
				lineSensor->lineFollowForwards();
			}
			else{
			   drivingChassis->stop();
               navState = FINISHED;
			}
			break;
		case FINISHED:
			navState = INITIALIZE_NAVIGATION;
			return true;
		}
	return false;
}



