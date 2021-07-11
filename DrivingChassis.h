/*
 * DrivingChassis.h
 *
 *  Created on: Jan 12, 2019
 *      Author: hephaestus
 */

#ifndef DRIVINGCHASSIS_H_
#define DRIVINGCHASSIS_H_
#include "src/pid/PIDMotor.h"
#include "src/commands/GetIMU.h"
#include "config.h"
#include "Pose.h"

#define WHEEL_DEGREES_TO_BODY_DEGREES 4.25F
#define MM_TO_WHEEL_DEGREES 2.1174F
#define WHEEL_DEGREES_TO_MM .472277F
#define MAX_SPEED_MM_PER_SEC 150
#define MAX_MOTOR_EFFORT_DURING_TURN 260 //300 //275 // 500


/**
 * @enum DrivingStatus
 * States when performing an drive action.
 */
enum DrivingStatus {
	REACHED_SETPOINT = 0,
	TIMED_OUT = 1,
	GOING_TO_SETPOINT = 2,
};

/**
 * @enum MotionType
 * States when performing an drive action.
 */
enum MotionType {
	DRIVING_FORWARDS = 0,
	DRIVING_BACKWARDS = 1,
	TURNING = 2,
};

/**
 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
 *
 * The 0,0,0 center of the robot is on the ground, half way between the left and right wheel contact points.
 *
 * The +X axis is the positive direction of travel
 *
 * The +Y axis goes from the center of the robot to the left wheel
 *
 * The +Z axis goes from the center up through the robot towards the ceiling.
 *
 * This object should manage the setting of motor setpoints to enable driving
 */
class DrivingChassis {
private:
	GetIMU * IMU;
	float mywheelTrackMM;
	float mywheelRadiusMM;
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
	float distanceToWheelAngle(float distance);
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
	float chassisRotationToWheelDistance(float angle);
public:
	// moved these over for line following
	PIDMotor * myleft;
	PIDMotor * myright;
	bool performingMovement = false;
	MotionType motionType;
	unsigned long startTimeOfMovement_ms;
	float wheelMovementKp = 3.5;// was 3.9
	float turningMovementKp = 21; //was 9, 11.7, 17.5
	float wheelMovementDeadband_mm = 2.5;
	float wheelMovementDeadband_deg = .5;
	float motionSetpoint;
	float timeout_ms;
	Pose myChassisPose;

	virtual ~DrivingChassis();

	/**
	 * DrivingChassis encapsulates a 2 wheel differential steered chassis that drives around
	 *
	 * @param left the left motor
	 * @param right the right motor
	 * @param wheelTrackMM is the measurment in milimeters of the distance from the left wheel contact point to the right wheels contact point
	 * @param wheelRadiusMM is the measurment in milimeters of the radius of the wheels
	 * @param imu The object that is used to access the IMU data
	 */
	DrivingChassis(PIDMotor * left, PIDMotor * right, float wheelTrackMM,
			float wheelRadiusMM,GetIMU * imu);

	/**
	 * Start a drive backwards action using the encoders and setpoint interpolation
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive backwards
	 * @param msDuration is the time in miliseconds that the drive action should take
	 *
	 * @note this function is fast-return and should not block
	 */
	void driveBackwardsFromInterpolation(float mmDistanceFromCurrent, int msDuration);

	/**
	 * Start a drive backwards action. Will return false when finished moving. Will stop when reached setpoint or timeout.
	 * Specifying a mmDistanceFromCurrent of '0' will move the robot indefinitely.
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive backwards
	 * @param msDuration is the time in miliseconds that the drive action should take (this is a timeout)
	 *
	 * @note this function is fast-return and should not block. Whatever is calling this should repeatedly do so
	 * until this function returns false due to reaching the set-point or timing out.
	 */
	void driveBackwards(float mmDistanceFromCurrent, int msDuration);

	/**
	 * Start a drive forward action using the encoders and setpoint interpolation
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
	 * @param msDuration is the time in miliseconds that the drive action should take
	 *
	 * @note this function is fast-return and should not block
	 */
	void driveForwardFromInterpolation(float mmDistanceFromCurrent, int msDuration);

	/**
	 * Start a drive forwards action. Will return false when finished moving. Will stop when reached setpoint or timeout.
	 * Specifying a mmDistanceFromCurrent of '0' will move the robot indefinitely.
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive forward
	 * @param msDuration is the time in miliseconds that the drive action should take (this is a timeout)
	 *
	 * @note this function is fast-return and should not block. Whatever is calling this should repeatedly do so
	 * until this function returns false due to reaching the set-point or timing out. Make sure to change state when this reaches a setpoint.
	 * Otherwise, this will try to reach a new setpoint (imagine driving indefinitely)
	 */
	void driveForward(float mmDistanceFromCurrent, int msDuration);

	/**
	 * Start a turn action using the encoders and setpoint interpolation
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
	void turnDegreesFromInterpolation(float degreesToRotateBase, int msDuration);

	/**
	 * Start a turn action. Will return false when finished moving. Will stop when reached setpoint or timeout.
	 *
	 * This action rotates the robot around the center line made up by the contact points of the left and right wheels.
	 * Positive angles should rotate to the left
	 *
	 * This rotation is a positive rotation about the Z axis of the robot.
	 *
	 * @param desiredHeading is the desired angle the robot should reach
	 * @param msDuration is the time in miliseconds that the drive action should take (this is a timeout)
	 *
	 * @note this function is fast-return and should not block. Whatever is calling this should repeatedly do so
	 * until this function returns false due to reaching the set-point or timing out.
	 */
	void turnToHeading(float desiredHeading, int msDuration);

	/**
	 * Check to see if the chassis is performing an action
	 *
	 * @return false is the chassis is driving, true is the chassis msDuration has elapsed
	 *
	 * @note this function is fast-return and should not block
	 */
	DrivingStatus statusOfChassisDriving();

	/**
	 * Stops all motors
	 */
	void stop();

	void driveStraight(float targetHeading, MotionType direction);

	/**
	 * loop()
	 *
	 * a fast loop function that will update states of the motors based on the information from the
	 * imu.
	 *
	 * @note this function is fast-return and should not block
	 */
	bool loop();

};

#endif /* DRIVINGCHASSIS_H_ */
