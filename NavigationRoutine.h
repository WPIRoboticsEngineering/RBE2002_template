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
	FINISHED = 6,
};

bool navigate(int row, int col, DrivingChassis* drivingChassis, LineFollower* lineSensor);

static NavigationStates navState = INITIALIZE_NAVIGATION;

#endif /* NAVIGATIONROUTINE_H_ */
