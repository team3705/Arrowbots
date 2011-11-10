#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */

//defines the constants (variables that don't change)

//represent voltage outputted by the potentiometer at certain positions, measured using two outside pins
#define kBottomRow ((float)(1.1))
#define kMiddleRow ((float)(2.59))
#define kTopRow ((float)(4.52))
#define kBottomRowSecondColumn ((float)(1.35))
#define kMiddleRowSecondColumn ((float)(2.86))
#define kTopRowSecondColumn ((float)(4.7))

#define kReleaseVoltage ((float) (0.1)) //* 	//voltage by which the shoulder should drop during releaseTubeOnPeg procedure
#define kPackingPositionShoulder ((float) (0.72))
#define kPackingPositionArm ((float) (4.06)) //*
#define kPickUpPosition ((float) kPackingPositionShoulder)	// same as klowestsafeshoulderpotreading
#define kLowestSafeShoulderPotReading ((float) kPickUpPosition) 

#define kGripperOpenPosition ((float)(4.3)) //**
#define kGripperClosePosition ((float)(2.6)) //**
#define kPerpendicularToFloor ((float)2.35) //*
#define kParallelToFloor ((float)4.21) //*

//miscellaneous constants
#define kMaxShoulderMotorSpeed ((float) 0.15)
#define kMaxArmMotorSpeed ((float) 0.6)
#define kMaxGripperMotorSpeed ((float) 0.6)

#define kNumberOfJoysticks ((int) 3)
#define kAutonomousDistance ((float) 85) //in inches

//constants used for shoulder motor movement calculations
/*
 #define kMaxShoulderRotation ((float) 2.0944) //in radians, 120 degrees
 #define kArmLength ((int) 66) //in inches; used as the 'radius' in arc length calculations
 #define	kShoulderLength ((int) 57) //in inches
 #define kTotalArcLength ((float) kArmLength*kMaxShoulderRotation)	//length that arm travels going from min potentiometer voltage to max
 #define pi ((float)3.14159265)
 */

class RobotDemo : public SimpleRobot {
	// declare your variables here. Pointers are used to decalre objects
	// It is in the format: Class *Variable

	RobotDrive *myRobot; // robot drive system (tank drive) 
	Joystick *leftstick; // "leftstick" is a variable that points to the joystick object.
	Joystick *rightstick; // right joystick
	Joystick *controlstick; //joystick used for controlling arm
	AnalogChannel *shoulderPotentiometerChannel;
	AnalogChannel *armPotentiometerChannel;
	AnalogChannel *gripperPotentiometerChannel;
	Encoder *wheelEncoder;

	DigitalInput *left; // used to get 0 or 1 from light sensors
	DigitalInput *middle;
	DigitalInput *right;

	RobotDrive *shoulderMotor;
	RobotDrive *armMotor;
	Victor *gripperMotor;

	//important variables declared here
	float shoulderPotentiometerReading; //variable used to hold shoulder actuator's potentiometer reading
	float armPotentiometerReading; //variable used to hold arm potentiometer reading
	float gripperPotentiometerReading; //variable used to hold gripper potentiometer reading
	int holdButton [kNumberOfJoysticks*11 + 1]; //since there are 11 buttons on each joystick; this array is used to hold a button during the method moveShoulderArmToHeight
	int joystickHolder [kNumberOfJoysticks + 1];
	double encoderReading;
	bool pickUp;
	bool movingArm;
	bool gripperOpen;

	//method variables below
	int buttonCounter;

public:
	// The class constructor, called once uopn bootup to initialize variables
	RobotDemo(void)

	{
		/*
		 * these must be initialized in the same order
		 * as they are declared above.
		 */
		myRobot = new RobotDrive(1, 2); // drive using motors connected to PWM 1 and 2
		leftstick = new Joystick(1); // leftstick = joystick on usb port # 1
		rightstick = new Joystick(2);
		controlstick = new Joystick(3);
		shoulderPotentiometerChannel = new AnalogChannel(1);
		armPotentiometerChannel = new AnalogChannel(2);
		gripperPotentiometerChannel = new AnalogChannel(3);

		//wheel encoder initialization
		wheelEncoder = new Encoder (4,5); //4th parameter: 0 means k1x, 1 means k2x, 2 means k4x 
		wheelEncoder->Start();
		wheelEncoder->Reset();
		wheelEncoder->SetDistancePerPulse(0.0359039085);

		left = new DigitalInput(1); // these DigitalInput instances are used to get 0 or 1 from light sensors
		middle = new DigitalInput(2); // connected to digital sidecar's channel 1, 2, and 3
		right = new DigitalInput(3);

		shoulderMotor = new RobotDrive (3, 10); //drive using motor connected to PWM 3 and the unused PWM 10
		armMotor = new RobotDrive (5, 9); //drive using motor connected to PWM 3 and the unused PWM 10
		gripperMotor = new Victor (4); //drive using motor connected to PWM 5


		//set the variables declared earlier below
		buttonCounter = 1;

		GetWatchdog().SetExpiration(0.1);
		/*a watchdog is an "inbuilt" system that monitors your code to 
		 * ensure that everything is running the way it is supposed to.
		 * To do this, the watchdog must be "fed" every once in a while.
		 * If not, it will die, and disable the code*/
	}

	/**
	 * Autonomous code
	 */
	void Autonomous(void) {

		//initialize watchdog and timer
		GetWatchdog().SetEnabled(false);
		Timer *lineFollowingTimer;
		lineFollowingTimer->Start();
		lineFollowingTimer->Reset();

		//variables used to hold light sensors' values
		int rightSensor = 0;
		int leftSensor = 0;
		int middleSensor = 0;
		double speed = 0.15; //enough speed to get to the peg in time as shoulder rises slowly
		bool reachedPosition = false;

		//run this loop for 15 seconds - DON'T CHANGE THIS - it will affect speed values
		while (lineFollowingTimer->Get() <= 15) {

			/*
			 //set speed depending on time
			 speed = 1-((lineFollowingTimer->Get())/10);
			 
			 if (speed < 0) {
			 speed = 0.0;
			 }
			 
			 //ensures speed does not go beyond max - CHANGE THIS TO THE MAX. SPEED YOU WANT AT THE BEGINNING
			 if (speed >= 0.25) {
			 speed = 0.25;
			 }
			 */

			//read distance wheel has traveled
			encoderReading = wheelEncoder->GetDistance();
			shoulderPotentiometerReading = (5
					-(shoulderPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1

			if (DistanceTraveled(kAutonomousDistance) == false) {

				// LINE FOLLOWING CODE - do this because we have not traveled enough distance yet
				leftSensor = left->Get() ? 1 : 0;
				middleSensor = middle->Get() ? 1 : 0;
				rightSensor = right->Get() ? 1 : 0;

				if (leftSensor == 0 && middleSensor == 1 && rightSensor == 1) {
					myRobot->Drive(speed, -0.5); // right and middle sensors are on line			
				} else if (leftSensor == 1 && middleSensor == 1 && rightSensor
						== 0) {
					myRobot->Drive(speed, 0.5); // left and middle sensors are on line	
				} else if ((leftSensor == 1 && middleSensor == 1 && rightSensor
						== 1) || (leftSensor == 1 && middleSensor == 0
						&& rightSensor == 1) || (leftSensor == 1
						&& middleSensor == 0 && rightSensor == 0)) {
					myRobot->Drive(speed, 0.9); // all on line / turn left if at the 'y'
				} else if (leftSensor == 0 && middleSensor == 0 && rightSensor
						== 0) {
					//CHANGE REQUIRED HERE WHEN DONE TESTING AUTONOMOUS
					myRobot->Drive(0.0, 0); // robot is off the line, reverse a bit at speed -0.1	
				} else if (leftSensor == 0 && middleSensor == 1 && rightSensor
						== 0) {
					myRobot->Drive(speed, 0.0); //only middle sensor is on line
				}

				MoveShoulderTo(kMiddleRow, 2, 10, false); //move shoulder while moving
			} else {
				//have traveled to the peg, stop then go to top row preset and open griper and drop shoulder a bit
				myRobot->Drive(0.0, 0);

				if (VoltageReached(shoulderPotentiometerReading, kMiddleRow)
						== 0 || reachedPosition == true) {
					//shoulder is now at the desired position
					reachedPosition = true;
					MoveGripperTo(kGripperOpenPosition);
					MoveShoulderTo(kMiddleRow-kReleaseVoltage, 2, 10, false); //lower the shoulder a bit
				} else {
					MoveShoulderTo(kMiddleRow, 2, 10, false); //move shoulder while moving
				}
			}
		}
		myRobot->Drive(0.0, 0); // stop robot	

	}

	/*
	 * OPERATOR CONTROL using a arcade style drive
	 */
	void OperatorControl(void) {

		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl()) //This code will loop continuously as long it is operator control mode
		{
			GetWatchdog().Feed(); // Feed the watchdog	

			if (rightstick->GetRawButton(1) == true) {
				//back of the robot is moved forward by pushing forward on the joystick
				myRobot->ArcadeDrive((rightstick->GetY()), (rightstick->GetX())
						/2, false); // drive with arcade drive
			} else {
				//front of the robot is moved forward by pushing forward on the joystick
				myRobot->ArcadeDrive(-(rightstick->GetY()),
						(rightstick->GetX())/2, false); // drive with arcade drive
			}

			shoulderPotentiometerReading = (5
					-(shoulderPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1
			armPotentiometerReading = (5
					-(armPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1
			gripperPotentiometerReading = (5
					-(gripperPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1
			encoderReading = wheelEncoder->GetDistance();

			//NOTES
			// Analog module must be in slot 1 of cRIO
			// USE LEFTSTICK BUTTON # 3 FOR RELEASE TUBE PROCEDURE
			// Test shoulder presets before recording and testing arm presets
			// Make sure to measure 40 inches from the ground to the tongue of gripper to ensure feeder position is correct
			//----------------------------------------------------------------------------


			//----------------------------------------------------------------------------


			//GRIPPER OPEN AND CLOSE
			
			
			if (leftstick->GetRawButton(1) == true || holdButton [1] == true) {
				if (gripperOpen == true) {
					if (VoltageReached (gripperPotentiometerReading, kGripperClosePosition) == 0) {
						holdButton [1] = false;
						gripperOpen = false;
						gripperMotor->Set(0);
					}
					else {
						holdButton [1] = true;
						MoveGripperTo(kGripperClosePosition);
					}
				}
				else {		
					if (VoltageReached (gripperPotentiometerReading, kGripperOpenPosition) == 0) {
						holdButton [1] = false;
						gripperOpen = true;
						gripperMotor->Set(0);
					}
					else {
						holdButton [1] = true;
						MoveGripperTo(kGripperOpenPosition);
						
					}
				}
				
			} 
			

			
			
			

			// POTENTIOMETER PRESETS
			// MoveShoulderTo (float potentiometerVoltage, int stickNumber (1 for left, 2 for right), int buttonNumber, bool waitForButtonToBePressed)

			//in case a wrong preset a chosen, press 2 to cancel the preset
			if (leftstick->GetRawButton(2) == true) {
				ReleaseAllButtonsOnJoystickAndStopShoulderMotor (1);
				ReleaseAllButtonsOnJoystickAndStopShoulderMotor (2);
			}
			MoveShoulderTo(kBottomRowSecondColumn, 1, 8, true); //when button # 8 on left joystick is pressed, move arm to bottom row, 2nd column
			MoveShoulderTo(kMiddleRowSecondColumn, 1, 7, true); //when button # 7 on left joystick is pressed, move arm to middle row, 2nd column
			MoveShoulderTo(kTopRowSecondColumn, 1, 6, true); //when button # 6 on left joystick is pressed, move arm to top row, 2nd column
			MoveShoulderTo(kBottomRow, 1, 9, true); //when button # 9 on left joystick is pressed, move arm to bottom row
			MoveShoulderTo(kMiddleRow, 1, 10, true); //when button # 10 on left joystick is pressed, move arm to middle row
			MoveShoulderTo(kTopRow, 1, 11, true); //when button # 11 on left joystick is pressed, move arm to top row


			//-------------------------------------------------------------------
			//MANUAL SHOULDER CONTROL - FOR GETTING POT. VOLTAGES
			if (leftstick->GetRawButton(4) == true) {
				shoulderMotor->Drive(-kMaxShoulderMotorSpeed, 0); //moves shoulderMotor down
				
			} else if (leftstick->GetRawButton(5) == true) {
				shoulderMotor->Drive(kMaxShoulderMotorSpeed,0); //moves shoulderMotor up
			}


			/*
			//MANUAL GRIPPER CONTROL
			if (leftstick->GetRawButton(1) == true) {
				MoveGripperTo(kGripperClosePosition);
			} else if (leftstick->GetRawButton(3) == true) {
				MoveGripperTo(kGripperOpenPosition);
			}
			else {
				gripperMotor->Set(0);
			}
			*/
			
			//MANUAL ARM CONTROL
			if (rightstick->GetRawButton(4) == true) {
				armMotor->Drive(-kMaxArmMotorSpeed/5.5, 0); //turn armMotor counter clockwise
			} else if (rightstick->GetRawButton(5) == true) {
				armMotor->Drive(kMaxArmMotorSpeed/1.5, 0); //turn armMotor cw
			}

			//PACK UP PROCEDURE - Hold till position is reached
			if (rightstick->GetRawButton(11) == true) {
				MoveShoulderTo(kPackingPositionShoulder, 2, 11, true); //when button # 11 on right joystick is pressed, move arm to packing robot position
				MoveArmTo(kPackingPositionArm);
			}
			

		}
	}

	bool DistanceTraveled (double distanceToReach) {

		distanceToReach = -distanceToReach;

		//check if distance has been reached
		if (encoderReading <= distanceToReach) {
			return true;
		}
		else if (encoderReading> distanceToReach) {
			return false;
		}
		else {
			return true;
		}
	}

	void MoveGripperTo(float gripperPotentiometerVoltageToReach) {

		
		if (VoltageReached (gripperPotentiometerReading, gripperPotentiometerVoltageToReach) == 1) {
			//voltage is less than needed,  turn motor cw
			gripperMotor->Set(kMaxGripperMotorSpeed);
		}
		else if (VoltageReached (gripperPotentiometerReading, gripperPotentiometerVoltageToReach) == 2) {
			//voltage is greater than needed,  turn motor ccw
			gripperMotor->Set(-kMaxGripperMotorSpeed);
		}
		else {
			gripperMotor->Set(0);
		}
		
	}

	void MoveArmTo(float armPotentiometerVoltageToReach) {

		if (VoltageReached(armPotentiometerReading, 5-armPotentiometerVoltageToReach) == 1) {
			armMotor->Drive(-kMaxArmMotorSpeed/6.5, 0);
		} else if (VoltageReached(armPotentiometerReading, 5-armPotentiometerVoltageToReach) == 2) {
			armMotor->Drive(kMaxArmMotorSpeed/1.5,0);
		} else {
			armMotor->Drive(0, 0);
		}

	}

	void MoveShoulderTo(float potentiometerVoltageToReach, int stickNumber, int buttonNumber, bool waitForButtonToBePressed) {

		//adjust the buttonNumber so that if stickNumber is 2, buttonNumber 12 will be checked if buttonNumber 1 was passed
		//11 is used because that is the number of buttons on a single joystick
		buttonNumber = ((stickNumber-1)*11)+buttonNumber;

		//repeat for any other joysticks
		if (waitForButtonToBePressed == false) {
			joystickHolder[1] = true;
			joystickHolder[2] = true;
			joystickHolder[3] = true;
		}
		else {
			//check if button has been pressed
			joystickHolder[1] = leftstick->GetRawButton(buttonNumber-((stickNumber-1)*11));
			joystickHolder[2] = rightstick->GetRawButton(buttonNumber-((stickNumber-1)*11));
			joystickHolder[3] = controlstick->GetRawButton(buttonNumber-((stickNumber-1)*11));
		}

		//depending on which joystick you want to check the button on, use the appropriate joystick
		//requires buttonNumber to allow it to continue rotating after button is released - it will continue to enter this if statement until desired position is reached 
		if (joystickHolder [stickNumber] == true || holdButton[buttonNumber] == true) { // If button # 1 on joytsick is pressed then (all buttons are numbered)
			if (VoltageReached(shoulderPotentiometerReading, potentiometerVoltageToReach) == 1) {
				shoulderMotor->Drive(kMaxShoulderMotorSpeed, 0); //move shoulder down

				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = true;
				}

				//put arm into parallel (scoring) position whenever a preset is selected
				if (potentiometerVoltageToReach != kPackingPositionShoulder && potentiometerVoltageToReach != kPickUpPosition) {
					//MoveArmTo (kParallelToFloor);
				}
			}
			else if (VoltageReached(shoulderPotentiometerReading, potentiometerVoltageToReach) == 2) {
				shoulderMotor->Drive(-kMaxShoulderMotorSpeed, 0); //move shoulder up

				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = true;
				}

				//put arm into parallel (scoring) position whenever a preset is selected
				if (potentiometerVoltageToReach != kPackingPositionShoulder && potentiometerVoltageToReach != kPickUpPosition) {
					//MoveArmTo (kParallelToFloor);
				}
			}
			else {
				//reached the desired position
				shoulderMotor->Drive(0,0); //stop shoulder

				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = false;
				}
			}

		}
	}

	void ReleaseAllButtonsOnJoystickAndStopShoulderMotor (int joystickNumber) {

		//sets all holdButton values for a certain joystick to false

		if (buttonCounter <= 11) {
			buttonCounter++;
			holdButton[(((joystickNumber-1)*11)+buttonCounter)] = false;
		}
		else {
			buttonCounter = 1;
		}
		return;
	}

	int VoltageReached (float voltage, float voltageToReach) {

		//0 = voltage has reached destination
		//1 = voltage is less than destination
		//2 = voltage is greater than destination
		//-1 = error

		if (voltage> voltageToReach-0.025 && voltage < voltageToReach+0.025) {
			return 0;
		}
		else if (voltage >= voltageToReach+0.025) {
			return 2;
		}
		else if (voltage <= voltageToReach-0.025) {
			return 1;
		}
		else {
			return -1;
		}

	}

};

START_ROBOT_CLASS(RobotDemo)
;
