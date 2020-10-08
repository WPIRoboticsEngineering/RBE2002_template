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
void DrivingChassis::driveForwardFromInterpolation(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not

	myleft -> overrideCurrentPosition(0);
	myright -> overrideCurrentPosition(0);
	myleft -> startInterpolationDegrees(mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
    myright -> startInterpolationDegrees(-mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
}

bool DrivingChassis::driveForward(float mmDistanceFromCurrent, int msDuration){
	// if we're not performing an action
	// start a timer, reset encoders
    if(!performingMovement){
    	startTimeOfMovement_ms = millis();
    	performingMovement = true;
    	myleft -> overrideCurrentPosition(0);
    	myright -> overrideCurrentPosition(0);
    	// TODO: call driveStraight method which uses IMU (if we're using an IMU)
    }
	// check for timeout
	if((millis() - startTimeOfMovement_ms) > msDuration){
		//timeout occured. Stop the robot
		Serial.println("Detected Timeout\r\n");
		stop();
		performingMovement = false;
		return false;
	}

    if(mmDistanceFromCurrent != -1){
    	float currentDistanceMovedRightWheel_mm = (myright -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
	    float currentDistanceMovedLeftWheel_mm = (myleft -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
	    float rightWheelError_mm = currentDistanceMovedRightWheel_mm - mmDistanceFromCurrent;
	    float leftWheelError_mm = - currentDistanceMovedLeftWheel_mm - mmDistanceFromCurrent;

	    if((fabs(rightWheelError_mm) < wheelMovementDeadband_mm) && (fabs(leftWheelError_mm) < wheelMovementDeadband_mm)){
		    Serial.println("Reached Setpoint \r\n");
		    stop();
		    performingMovement = false;
		    return false;
	    }
	    else{
		    //Serial.println("Right Error: " + String(rightWheelError_mm) + "\r\n" );
		    //Serial.println("Left Error: " + String(leftWheelError_mm) + "\r\n" );
		    myright -> setVelocityDegreesPerSecond(-wheelMovementKp*rightWheelError_mm);
		    myleft -> setVelocityDegreesPerSecond(wheelMovementKp*leftWheelError_mm);
	    }
    }

    else{
    	// sets speed to 10 cm per second
    	myright -> setVelocityDegreesPerSecond(100*MM_TO_WHEEL_DEGREES);
        myleft -> setVelocityDegreesPerSecond(-100*MM_TO_WHEEL_DEGREES);
    }

    return true;
}

/**
	 * Start a drive backwards action
	 *
	 * @param mmDistanceFromCurrent is the distance the mobile base should drive backwards
	 * @param msDuration is the time in miliseconds that the drive action should take
	 *
	 * @note this function is fast-return and should not block
*/
void DrivingChassis::driveBackwardsFromInterpolation(float mmDistanceFromCurrent, int msDuration) {
	// We should also have a "drive straight" method that uses the IMU to actually drive straight
	// Although we'll have a linefollow state so... idk maybe not
	myleft -> overrideCurrentPosition(0);
	myright -> overrideCurrentPosition(0);

	myleft -> startInterpolationDegrees(-mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
    myright -> startInterpolationDegrees(mmDistanceFromCurrent * MM_TO_WHEEL_DEGREES, msDuration, LIN);
}

bool DrivingChassis::driveBackwards(float mmDistanceFromCurrent, int msDuration){
		// if we're not performing an action
		// start a timer, reset encoders
	    if(!performingMovement){
	    	startTimeOfMovement_ms = millis();
	    	performingMovement = true;
	    	myleft -> overrideCurrentPosition(0);
	    	myright -> overrideCurrentPosition(0);
	    	// TODO: call driveStraight method which uses IMU (if we're using an IMU)
	    }
		// check for timeout
		if((millis() - startTimeOfMovement_ms) > msDuration){
			//timeout occured. Stop the robot
			Serial.println("Detected Timeout\r\n");
			stop();
			performingMovement = false;
			return false;
		}

	    if(mmDistanceFromCurrent != -1){
	    	float currentDistanceMovedRightWheel_mm = (myright -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
		    float currentDistanceMovedLeftWheel_mm = (myleft -> getAngleDegrees())*WHEEL_DEGREES_TO_MM;
		    float rightWheelError_mm = - currentDistanceMovedRightWheel_mm - mmDistanceFromCurrent;
		    float leftWheelError_mm = currentDistanceMovedLeftWheel_mm - mmDistanceFromCurrent;

		    if((fabs(rightWheelError_mm) < wheelMovementDeadband_mm) && (fabs(leftWheelError_mm) < wheelMovementDeadband_mm)){
			    Serial.println("Reached Setpoint \r\n");
			    stop();
			    performingMovement = false;
			    return false;
		    }
		    else{
			    //Serial.println("Right Error: " + String(rightWheelError_mm) + "\r\n" );
			    //Serial.println("Left Error: " + String(leftWheelError_mm) + "\r\n" );
			    myright -> setVelocityDegreesPerSecond(wheelMovementKp*rightWheelError_mm);
			    myleft -> setVelocityDegreesPerSecond(-wheelMovementKp*leftWheelError_mm);
		    }
	    }

	    else{
	    	// sets speed to 10 cm per second
	    	myright -> setVelocityDegreesPerSecond(-100*MM_TO_WHEEL_DEGREES);
	        myleft -> setVelocityDegreesPerSecond(100*MM_TO_WHEEL_DEGREES);
	    }

	    return true;
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
void DrivingChassis::turnDegreesFromInterpolation(float degreesToRotateBase, int msDuration) {
	   myleft -> overrideCurrentPosition(0);
	   myright -> overrideCurrentPosition(0);
	   myleft -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
	   myright -> startInterpolationDegrees(degreesToRotateBase * WHEEL_DEGREES_TO_BODY_DEGREES, msDuration, LIN);
}

bool DrivingChassis::turnToHeading(float degreesToRotateBase, int msDuration){
	/* As of 10/4/2020 Gabe doesn't have an IMU... rippu
	  Two variants, one with IMU and one without IMU.
	  IMU variant: will use P controller to modulate speed to make the turn based on
	  heading. Maybe we can even make this absolute in the future.
	  Encoder variant: Make the turn based on encoder ticks
	  */
	 if(!performingMovement){
		    	startTimeOfMovement_ms = millis();
		    	performingMovement = true;
     }
	 // check for timeout
     if((millis() - startTimeOfMovement_ms) > msDuration){
			//timeout occured. Stop the robot
			Serial.println("Detected Timeout\r\n");
			stop();
			performingMovement = false;
			return false;
    }

	float currentHeading = IMU->getEULER_azimuth();

	float headingError = - currentHeading - degreesToRotateBase;

	if(fabs(headingError) <= wheelMovementDeadband_deg)
	{
		Serial.println("Reached Setpoint\r\n");
		performingMovement = false;
		stop();
		return false;
	}
	else{
		    Serial.println("Heading Error: " + String(headingError) +"\r\n");
            myleft->setVelocityDegreesPerSecond(-(wheelMovementKp*2.8) * headingError);
            myright->setVelocityDegreesPerSecond(-(wheelMovementKp*2.8) * headingError);
	}
	return true;
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

void DrivingChassis::stop(){
	myleft -> stop();
	myright -> stop();
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
