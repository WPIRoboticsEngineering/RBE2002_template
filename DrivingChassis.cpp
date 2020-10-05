/*
 * DrivingChassis.cpp
 *
 *  Created on: Jan 31, 2019
 *      Author: hephaestus
 */

#include "DrivingChassis.h"

/**
 * Compute a delta in wheel angle to traverse a specific distance
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param distance a distance for this wheel to travel in MM
 * @return the wheel angle delta in degrees
 */
float DrivingChassis::distanceToWheelAngle(float distance) {
	return 0;
}

/**
 * Compute the arch length distance the wheel needs to travel through to rotate the base
 * through a given number of degrees.
 *
 * arc length	=	2*	Pi*	R*	(C/360)
 *
 * C  is the central angle of the arc in degrees
 * R  is the radius of the arc
 * Pi  is Pi
 *
 * @param angle is the angle the base should be rotated by
 * @return is the linear distance the wheel needs to travel given the this CHassis's wheel track
 */
float DrivingChassis::chassisRotationToWheelDistance(float angle) {
	return 0;
}

DrivingChassis::~DrivingChassis() {
	// do nothing
}

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * @param left the left motor
 * @param right the right motor
 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
 * @param imu The object that is used to access the IMU data
 *
 */
DrivingChassis::DrivingChassis(PIDMotor * left, PIDMotor * right,
		float wheelTrackMM, float wheelRadiusMM,GetIMU * imu) {
	myleft = left;
	myright = right;
	mywheelTrackMM = wheelTrackMM;
	mywheelRadiusMM = wheelRadiusMM;
	IMU = imu;
}

/**
 * Start a drive forward action
 *
 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 * @note this function is fast-return and should not block
 * @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		 allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not
	myleft -> startInterpolationDegrees(mmDistanceFromCurrent * WHEEL_DEGREES_TO_MM, msDuration, LIN);
    myright -> startInterpolationDegrees(-mmDistanceFromCurrent * WHEEL_DEGREES_TO_MM, msDuration, LIN);
}

/**
	 * Start a drive backwards action
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive backwards
	 * @param msDuration is the time in miliseconds that the drive action should take
	 *
	 * @note this function is fast-return and should not block
*/
void DrivingChassis::driveBackwards(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not
	myleft -> startInterpolationDegrees(-mmDistanceFromCurrent * WHEEL_DEGREES_TO_MM, msDuration, LIN);
    myright -> startInterpolationDegrees(mmDistanceFromCurrent * WHEEL_DEGREES_TO_MM, msDuration, LIN);
}

/**
 * Start a turn action
 *
 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
 * Positive angles should rotate to the left
 *
 * This rotation is a positive rotation about the Z axis of the robot.
 *
 * @param degreesToRotateBase the number of degrees to rotate
 * @param msDuration is the time in miliseconds that the drive action should take
 *
 *  @note this function is fast-return and should not block
 *  @note pidmotorInstance->overrideCurrentPosition(0); can be used to "zero out" the motor to
 * 		  allow for relative moves. Otherwise the motor is always in ABSOLUTE mode
 */
void DrivingChassis::turnDegrees(float degreesToRotateBase, int msDuration) {
	/* As of 10/4/2020 Gabe doesn't have an IMU... rippu
	  Two variants, one with IMU and one without IMU.
	  IMU variant: will use P controller to modulate speed to make the turn based on
	  heading. Maybe we can even make this absolute in the future.
	  Encoder variant: Make the turn based on encoder ticks
	  */
	#ifdef USE_IMU
	#else
	   myleft -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
	   myright -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
	#endif
}

/**
 * Check to see if the chassis is performing an action
 *
 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
 *
 * @note this function is fast-return and should not block
 */
bool DrivingChassis::isChassisDoneDriving() {
	return false;
}
/**
 * loop()
 *
 * a fast loop function that will update states of the motors based on the information from the
 * imu.
 */
bool DrivingChassis::loop(){
return false;
}
