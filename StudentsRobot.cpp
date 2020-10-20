/*
 * StudentsRobot.cpp
 *
 *  Created on: Dec 28, 2018
 *      Author: hephaestus
 */


#include "StudentsRobot.h"

uint32_t startTime = 0;

StudentsRobot::StudentsRobot(PIDMotor * motor1, PIDMotor * motor2,
		PIDMotor * motor3, Servo * servo, IRCamSimplePacketComsServer * IRCam,
		GetIMU * imu): robotChassis(motor2, motor1, 230, 30, imu), lineSensor(&robotChassis) {
	Serial.println("StudentsRobot::StudentsRobot constructor called here ");

	this->servo = servo;
	this->motor1 = motor1;
	this->motor2 = motor2;
	this->motor3 = motor3;
	IRCamera = IRCam;
	IMU = imu;
#if defined(USE_IMU)
	IMU->setXPosition(200);
	IMU->setYPosition(0);
	IMU->setZPosition(0);
#endif
	// Set the PID Clock gating rate. The PID must be 10 times slower than the motors update rate
	motor1->myPID.sampleRateMs = 5; //
	motor2->myPID.sampleRateMs = 5; //
	motor3->myPID.sampleRateMs = 5;  // 10khz H-Bridge, 0.1ms update, 1 ms PID

	// Set default P.I.D gains
	motor1->myPID.setpid(0.00015, 0, 0);
	motor2->myPID.setpid(0.00015, 0, 0);
	motor3->myPID.setpid(0.00015, 0, 0);

	motor1->velocityPID.setpid(0.1, 0, 0);
	motor2->velocityPID.setpid(0.1, 0, 0);
	motor3->velocityPID.setpid(0.1, 0, 0);
	// compute ratios and bounding
	double motorToWheel = 3;
	motor1->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor2->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					motorToWheel * // motor to wheel stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			480, // measured max degrees per second
			150	// the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	motor3->setOutputBoundingValues(-255, //the minimum value that the output takes (Full reverse)
			255, //the maximum value the output takes (Full forward)
			0, //the value of the output to stop moving
			125, //a positive value subtracted from stop value to creep backward
			125, //a positive value added to the stop value to creep forwards
			16.0 * // Encoder CPR
					50.0 * // Motor Gear box ratio
					1.0 * // motor to arm stage ratio
					(1.0 / 360.0) * // degrees per revolution
					2, // Number of edges that are used to increment the value
			1400, // measured max degrees per second
			50 // the speed in degrees per second that the motor spins when the hardware output is at creep forwards
			);
	// Set up the Analog sensors
	pinMode(LEFT_LINE_SENSOR, ANALOG);
	pinMode(RIGHT_LINE_SENSOR, ANALOG);
	pinMode(ANALOG_SENSE_THREE, ANALOG);
	pinMode(ANALOG_SENSE_FOUR, ANALOG);
	// H-Bridge enable pin
	pinMode(H_BRIDGE_ENABLE, OUTPUT);
	// Stepper pins
	pinMode(STEPPER_DIRECTION, OUTPUT);
	pinMode(STEPPER_STEP, OUTPUT);
	// User button
	pinMode(BOOT_FLAG_PIN, INPUT_PULLUP);
	//Test IO
	pinMode(WII_CONTROLLER_DETECT, OUTPUT);
}
/**
 * Seperate from running the motor control,
 * update the state machine for running the final project code here
 */
void StudentsRobot::updateStateMachine() {
	digitalWrite(WII_CONTROLLER_DETECT, 1);
	long now = millis();
	switch (status) {
	case StartupRobot:
		//Do this once at startup
		status = StartRunning;
		Serial.println("StudentsRobot::updateStateMachine StartupRobot here ");
		break;
	case StartRunning:
		Serial.println("Start Running");

		digitalWrite(H_BRIDGE_ENABLE, 1);
		// Start an interpolation of the motors
		motor1->startInterpolationDegrees(motor1->getAngleDegrees(), 1000, SIN);
		motor2->startInterpolationDegrees(motor2->getAngleDegrees(), 1000, SIN);
		motor3->startInterpolationDegrees(motor3->getAngleDegrees(), 1000, SIN);
		status = WAIT_FOR_MOTORS_TO_FINNISH; // set the state machine to wait for the motors to finish
		nextStatus = Running; // the next status to move to when the motors finish
		startTime = now + 1000; // the motors should be done in 1000 ms
		nextTime = startTime + 1000; // the next timer loop should be 1000ms after the motors stop
		break;
	case Running:
		// Set up a non-blocking 1000 ms delay
//		status = WAIT_FOR_TIME;
//		nextTime = nextTime + 1; // ensure no timer drift by incremeting the target
//		// After 1000 ms, come back to this state
//		nextStatus = Running;
		// Do something
		// On button press we go into testing state
		if (!digitalRead(BOOT_FLAG_PIN)) {
			Serial.println(
					" Running State Machine " + String((now - startTime)));
			//robotChassis.turnDegrees(-90, 5000);
			//robotChassis.driveForward(100, 5000);
			//robotChassis.driveBackwards(300, 5000);
#if defined(USE_IMU)
			IMU->print();
#endif
#if defined(USE_IR_CAM)
			IRCamera->print();
#endif
		// I put in this delay so that I have time to step back
		//status = WAIT_FOR_TIME;
		//nextTime = millis() + 3000; // ensure no timer drift by incremeting the target
		// After 1000 ms, come back to this state
		//nextStatus = TestingBasicMovement;
	    startTime = millis();
		status = Testing;
		}
		break;
	case WAIT_FOR_TIME:
		// Check to see if enough time has elapsed
		if (nextTime <= millis()) {
			// if the time is up, move on to the next state
			status = nextStatus;
		}
		break;
	case WAIT_FOR_MOTORS_TO_FINNISH:
		if (motor1->isInterpolationDone() && motor2->isInterpolationDone()
				&& motor3->isInterpolationDone()) {
			status = nextStatus;
		}
		break;
	case Halting:
		// save state and enter safe mode
		Serial.println("Halting State machine");
		digitalWrite(H_BRIDGE_ENABLE, 0);
		motor3->stop();
		motor2->stop();
		motor1->stop();

		status = Halt;
		break;
	case Halt:
		// in safe mode
		break;
	case Testing:
// CLAMPED CONTROL
//		DrivingStatus statusOfForward = robotChassis.turnToHeading(90, 2000);
//		if(statusOfForward == REACHED_SETPOINT){
//			status = Running;
//		}
//		else if(statusOfForward == TIMED_OUT){
//			status = Running;
//		}
/// LINE FOLLOWING
		static bool foundCol = false;
		static bool turnedToBin = false;
		if(millis() - startTime < 10000){
			lineSensor.lineFollow();
//			if(!foundCol){
//			    lineSensor.lineFollow();
//			    if(lineSensor.lineCount == 2){
//			    	foundCol = true;
//			    }
//			}
//			if(foundCol && !turnedToBin){
//				if(robotChassis.turnToHeading(90, 1000) == REACHED_SETPOINT){
//					turnedToBin = true;
//				}
//			}
////		    int leftSensorValue = analogRead(LEFT_LINE_SENSOR);
////		    int rightSensorValue = analogRead(RIGHT_LINE_SENSOR);
////
////		    Serial.println(String(leftSensorValue) + " " + String(rightSensorValue) + "\r\n");
		}
		else{
		   Serial.println("Line Count:" + String(lineSensor.lineCount) + "\r\n");
		   robotChassis.stop();
		   lineSensor.resetLineCount();
		   foundCol = false;
		   turnedToBin = false;
		   status = Running;
		}

// BASIC MOTION
/*
		static bool movedForward = false;
		static bool movedBack    = false;
	    static bool turnedRight  = false;
		static bool turnedLeft   = false;
		static bool backToZero   = false;

		if(!movedForward){
			if(robotChassis.driveForward(300, 5000) == REACHED_SETPOINT){
				movedForward = true;
			}
		}
		else if(movedForward && !movedBack){
				if(robotChassis.driveBackwards(300, 5000) == REACHED_SETPOINT){
					movedBack = true;
				}
	    }
		else if(movedBack && !turnedRight){
			if(robotChassis.turnToHeading(-90, 5000) == REACHED_SETPOINT){
				turnedRight = true;
			}
		}
		else if(turnedRight && !turnedLeft){
			if(robotChassis.turnToHeading(90, 5000) == REACHED_SETPOINT){
				turnedLeft = true;
			}
		}
		else if(turnedLeft && !backToZero){
			if(robotChassis.turnToHeading(0, 5000) == REACHED_SETPOINT){
				backToZero = true;
				status = Running;
				movedBack = false;
				movedForward = false;
				turnedLeft = false;
				turnedRight = false;
				backToZero  = false;
			}
		}
*/
		break;

	}
	digitalWrite(WII_CONTROLLER_DETECT, 0);
}


