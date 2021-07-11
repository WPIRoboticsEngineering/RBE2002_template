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
	   int lineFollowingSpeedForwards_mm_per_sec = 150;
	   int lineFollowingSpeedBackwards_mm_per_sec = 175;
	   float lineFollowingKpBackwards = .002*lineFollowingSpeedBackwards_mm_per_sec;
	   float lineFollowingKpForwards = 1.3; //1.6
	   int lineCount = 0;
       DrivingChassis* robotChassis;
       bool canCountLine = true;

	   void lineFollowBackwards();
	   void lineFollowForwards();
	   void resetLineCount();

    private:
};




#endif /* LINEFOLLOWER_H_ */
