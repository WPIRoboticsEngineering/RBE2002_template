/*
 * NavigationRoutine.h
 *
 *  Created on: Oct 22, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "LineFollower.h"

#ifndef NAVIGATIONROUTINE_H_
#define NAVIGATIONROUTINE_H_

enum NavigationStates{
	INITIALIZE_NAVIGATION = 0,
	TURN_TOWARDS_CORRECT_COLUMN = 1,
	FINDING_OUTER_EDGE = 2,
	FINDING_ROW = 3,
	TURN_TOWARDS_CORRECT_ROW = 4,
	FINDING_COLUMN = 5,
	WAIT_FOR_MOTION_SETPOINT_REACHED = 6,
	FINISHED = 7,
};

void setNavGoal(int row, int col, DrivingChassis* robotChassis, LineFollower* lineFollower);
NavigationStates checkNavStatus();

static DrivingChassis* drivingChassis;
static LineFollower* lineSensor;
static int goalRow;
static int goalCol;

static NavigationStates navState = INITIALIZE_NAVIGATION;

// This is the navState that occurs after a setpoint has been reached
static NavigationStates navStateAfterMotionSetpointReached;

#endif /* NAVIGATIONROUTINE_H_ */
