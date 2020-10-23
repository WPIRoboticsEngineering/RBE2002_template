/*
 * Pose.h
 *
 *  Created on: Oct 21, 2020
 *      Author: Gabe's PC
 */
#ifndef POSE_H_
#define POSE_H_

class Pose{
    public:
	   Pose();
	   // current row and current column will be updated by the line sensor lineCount
	   int currentRow = 0;
	   int currentColumn = 0;
	   // current heading will be updated by the IMU during turnToHeading.
	   float heading = 0;
	   int rowCount = 0;
	   int colCount = 0;

    private:
};



#endif /* POSE_H_ */
