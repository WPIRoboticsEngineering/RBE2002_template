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
	   const int ON_BLACK = 3500;
	   int lineCount = 0;
	   bool canCountLine = true;
       DrivingChassis* robotChassis;

	   void lineFollow();
	   void resetLineCount();

    private:
};




#endif /* LINEFOLLOWER_H_ */
