/*
 * LineFollower.h
 *
 *  Created on: Oct 15, 2020
 *      Author: gentov
 */

#include "DrivingChassis.h"
#include "config.h"
#ifndef LINEFOLLOWER_H_
#define LINEFOLLOWER_H_


class LineFollower{
    public:
	   LineFollower(DrivingChassis* myChassis);
	   // on black line
	   const int ON_BLACK = 3692;//3750;
	   int lineFollowingSpeed_mm_per_sec = 175;
	   float lineFollowingKp = .002*lineFollowingSpeed_mm_per_sec;
	   int lineCount = 0;
	   bool canCountLine = true;
       DrivingChassis* robotChassis;

	   void lineFollow();
	   void resetLineCount();

    private:
};




#endif /* LINEFOLLOWER_H_ */
