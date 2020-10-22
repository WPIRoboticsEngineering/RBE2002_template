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
//		if(lineSensor.lineCount == 0){
//			lineSensor.lineFollow();
//		}
//		else{
//		   if(robotChassis.turnToHeading(180, 5000) == REACHED_SETPOINT){
//		   Serial.println("Line Count:" + String(lineSensor.lineCount) + "\r\n");
//		   Serial.println("Pose Row:" + String(robotChassis.myChassisPose.currentRow) + "\r\n");
//		   Serial.println("Pose Column:" + String(robotChassis.myChassisPose.currentColumn) + "\r\n");
//		   Serial.println("Pose Heading:" + String(robotChassis.myChassisPose.heading) + "\r\n");
//		   robotChassis.stop();
//		   lineSensor.resetLineCount();
//		   status = Running;
//		   }
//		}

// Navigation
		switch(navState){
		case INITIALIZE_NAVIGATION:
			Serial.println("INIT NAV");
			// if we're in the outerlane, then there really isn't a need to find the outerlane
			if(robotChassis.myChassisPose.currentColumn == 0){
				navState = TURN_TOWARDS_CORRECT_ROW;
			}
			else{
				navState = TURN_TOWARDS_CORRECT_COLUMN;
			}
			break;
		case TURN_TOWARDS_CORRECT_COLUMN:
			Serial.println("TURNING TOWARDS COLUMN");
			// determine what is our first state
		    if(robotChassis.myChassisPose.currentRow != goalRow){
				// if our current row isn't our goal row, then we need to navigate to the outer edge
				// first
		        /// TODO: check where we are, maybe our orientation is correct
			    if(robotChassis.turnToHeading(90, 2500) == REACHED_SETPOINT){
				     navState = FINDING_OUTER_EDGE;
			    }
			}
			else{
				// otherwise, we just need to find the right column
				if(robotChassis.myChassisPose.currentColumn > goalColumn){
					if(robotChassis.turnToHeading(-90, 2500) == REACHED_SETPOINT){
						navState = FINDING_COLUMN;
					}
				}
				else{
					if(robotChassis.turnToHeading(90, 2500) == REACHED_SETPOINT){
						navState = FINDING_COLUMN;
					}
				}
			}
			break;
		case FINDING_OUTER_EDGE:
			Serial.println("FINDING COL: 0, CURRENT COL: " + String(robotChassis.myChassisPose.currentColumn));
		    // navigate until column == 0
			if(robotChassis.myChassisPose.currentColumn != 0){
				// if the column is wrong.
				lineSensor.lineFollow();
			}
			else{
                if(robotChassis.driveBackwards(210, 2500) == REACHED_SETPOINT){
                	robotChassis.stop();
                	navState = TURN_TOWARDS_CORRECT_ROW;
                }
			}
			break;
		case FINDING_ROW:
			Serial.println("FINDING ROW: " + String(goalRow) +  "CURRENT ROW: " + String(robotChassis.myChassisPose.currentRow));
			if(robotChassis.myChassisPose.currentRow != goalRow){
				// if the row is wrong.
				lineSensor.lineFollow();
			}
			else{
				robotChassis.stop();
				if(goalColumn != 0){
				    if(robotChassis.driveBackwards(210, 2500) == REACHED_SETPOINT){
					    navState = TURN_TOWARDS_CORRECT_COLUMN;
				    }
				}
			}
			break;
		case TURN_TOWARDS_CORRECT_ROW:
			Serial.println("TURNING TOWARDS ROW");
			// otherwise, we just need to find the right column
			if(robotChassis.myChassisPose.currentRow > goalRow){
				if(robotChassis.turnToHeading(180, 2500) == REACHED_SETPOINT){
					navState = FINDING_ROW;
				}
			}
			else{
				if(robotChassis.turnToHeading(0, 2500) == REACHED_SETPOINT){
					navState = FINDING_ROW;
				}
			}
			break;
		case FINDING_COLUMN:
			Serial.println("FINDING COL: " + String(goalColumn) +  "CURRENT COL: " + String(robotChassis.myChassisPose.currentColumn));
			if(robotChassis.myChassisPose.currentColumn != goalColumn){
				// if the row is wrong.
				lineSensor.lineFollow();
			}
			else{
				if(robotChassis.driveBackwards(210, 2500) == REACHED_SETPOINT){
					//goalRow = 2;
					//goalColumn = -1;
					goalRow = 3;
					goalColumn = 0;
					robotChassis.stop();
				    navState = INITIALIZE_NAVIGATION;
				}
			}
			break;
		case FINISHED:
			break;
		}
/// POSE TRACKING

//	   static int testCase = 1;
//       switch(testCase){
//          case 1:
//        	  if(robotChassis.myChassisPose.currentRow != 1){
//        	      lineSensor.lineFollow();
//        	  }
//        	  else{
//			      robotChassis.stop();
//			      testCase = 2;
//			  }
//        	  break;
//          case 2:
//        	  if(robotChassis.driveBackwards(200, 1500) == REACHED_SETPOINT){
//        		  testCase = 3;
//        		  lineSensor.canCountLine = true;
//        	  }
//        	  break;
//          case 3:
//        	  // turn right 90
//        	  if(robotChassis.turnToHeading(-90, 1500) == REACHED_SETPOINT){
//        		  testCase = 4;
//        	  }
//        	  break;
//          case 4:
//        	  // line follow until column
//        	  if(robotChassis.myChassisPose.currentColumn != -1){
//        	      lineSensor.lineFollow();
//        	  }
//        	  else{
//        	      robotChassis.stop();
//        	      testCase = 5;
//        	  }
//        	  break;
//          case 5:
//        	  if(robotChassis.turnToHeading(90, 1500) == REACHED_SETPOINT){
//        	      Serial.println("Pose Row:" + String(robotChassis.myChassisPose.currentRow) + "\r\n");
//        	      Serial.println("Pose Column:" + String(robotChassis.myChassisPose.currentColumn) + "\r\n");
//        	      Serial.println("Pose Heading:" + String(robotChassis.myChassisPose.heading) + "\r\n");
//        	      status = Running;
//        	      testCase = 1;
//        	  }
//        	  break;
//       }
// BASIC MOTION

//		static bool movedForward = false;
//		static bool movedBack    = false;
//	    static bool turnedRight  = false;
//		static bool turnedLeft   = false;
//		static bool backToZero   = false;
//
//		if(!movedForward){
//			if(robotChassis.driveForward(300, 5000) == REACHED_SETPOINT){
//				movedForward = true;
//			}
//		}
//		else if(movedForward && !movedBack){
//				if(robotChassis.driveBackwards(300, 5000) == REACHED_SETPOINT){
//					movedBack = true;
//				}
//	    }
//		else if(movedBack && !turnedRight){
//			if(robotChassis.turnToHeading(-90, 5000) == REACHED_SETPOINT){
//				turnedRight = true;
//			}
//		}
//		else if(turnedRight && !turnedLeft){
//			if(robotChassis.turnToHeading(90, 5000) == REACHED_SETPOINT){
//				turnedLeft = true;
//			}
//		}
//		else if(turnedLeft && !backToZero){
//			if(robotChassis.turnToHeading(0, 5000) == REACHED_SETPOINT){
//				backToZero = true;
//				status = Running;
//				movedBack = false;
//				movedForward = false;
//				turnedLeft = false;
//				turnedRight = false;
//				backToZero  = false;
//			}
//		}
		break;

	}
	digitalWrite(WII_CONTROLLER_DETECT, 0);
}


