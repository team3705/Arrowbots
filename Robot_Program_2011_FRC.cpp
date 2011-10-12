#include "WPILib.h"

/**
 * Programmer: Fawaz Tahir
 * Team 3705 ArrowBots
 * For 2011 Logomotion FIRST Robotics Competition
 */

//defines the constants (variables that don't change)

//represent voltage outputted by the potentiometer at certain positions, measured using two outside pins
#define kBottomRow ((float)(1.1))
#define kMiddleRow ((float)(2.59))
#define kTopRow ((float)(4.52))
#define kBottomRowSecondColumn ((float)(1.35))
#define kMiddleRowSecondColumn ((float)(2.86))
#define kTopRowSecondColumn ((float)(4.7))

#define kPackingPositionShoulder ((float) (1.0))
#define kPackingPositionArm ((float) (4.06)) //*
#define kPickUpPosition ((float) kPackingPositionShoulder)
#define kPerpendicularToFloor ((float)2.35) //*
#define kParallelToFloor ((float)4.21) //*
//miscellaneous constants
#define kMaxShoulderMotorSpeed ((float) 0.65)
#define kMaxArmMotorSpeed ((float) 0.85)
#define kNumberOfJoysticks ((int) 3)

class RobotDemo : public SimpleRobot {
	// declare your variables here. Pointers are used to decalre objects
	// It is in the format: Class *Variable

	RobotDrive *myRobot; // robot drive system (tank drive) 
	Joystick *leftstick; // "leftstick" is a variable that points to the joystick object.
	Joystick *rightstick; // right joystick
	AnalogChannel *shoulderPotentiometerChannel;

	DigitalInput *left; // used to get 0 or 1 from light sensors
	DigitalInput *middle;
	DigitalInput *right;

	RobotDrive *shoulderMotor;
	RobotDrive *armMotor;
	Servo *gripperMotor;
	
	Servo *minibotDeployMotor;
	Servo *minibotCloseMotor1;
	Servo *minibotCloseMotor2;

	//important variables declared here
	float shoulderPotentiometerReading; //variable used to hold shoulder actuator's potentiometer reading
	float shoulderDestinationVoltage;
	int rightSensor;
	int leftSensor;
	int middleSensor;
	Timer *gameTimer;

public:
	// The class constructor, called once upon bootup to initialize variables
	RobotDemo(void)

	{
		/*
		 * these must be initialized in the same order
		 * as they are declared above.
		 */
		myRobot = new RobotDrive(1, 2); // drive using motors connected to PWM 1 and 2
		leftstick = new Joystick(1); // leftstick = joystick on usb port # 1
		rightstick = new Joystick(2);
		shoulderPotentiometerChannel = new AnalogChannel(1);

		left = new DigitalInput(1); // these DigitalInput instances are used to get 0 or 1 from light sensors
		middle = new DigitalInput(2); // connected to digital sidecar's channel 1, 2, and 3
		right = new DigitalInput(3);

		shoulderMotor = new RobotDrive (3, 10); //drive using motor connected to PWM 3 and the unused PWM 10
		armMotor = new RobotDrive (5, 9); //drive using motor connected to PWM 5 and the unused PWM 9 
		gripperMotor = new Servo (4); //drive using motor connected to servo channel 1
										
		minibotDeployMotor = new Servo (6); //drive using motor connected to servo channel 2
		minibotCloseMotor1 = new Servo (7); //drive using motor connected to servo channel 3
		minibotCloseMotor2 = new Servo (8); //drive using motor connected to servo channel 3


		//set the variables declared earlier below
		shoulderDestinationVoltage = 5-shoulderPotentiometerChannel->GetVoltage();

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

		/*
		 //disable watchdog and start timer
		 GetWatchdog().SetEnabled(false);
		 gameTimer->Start();
		 gameTimer->Reset();

		 //variables used to hold light sensors' values
		 rightSensor = 0;
		 leftSensor = 0;
		 middleSensor = 0;

		 float speed = 0.15; //CHECK-> enough speed to get to the peg in time as shoulder rises slowly
		 float releaseVoltage;
		 bool reachedEndOfLine = false;
		 int followingLineNumber = 1; //2 is for the 'y' line; reading line number from left to right


		 //keep following line until robot reaches the 'T' - move shoulder to appropriate position simultaneously
		 while (reachedEndOfLine == false) {

		 //read the various sensors
		 //encoderReading = wheelEncoder->GetDistance();
		 shoulderPotentiometerReading = (5
		 -(shoulderPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1


		 // LINE FOLLOWING CODE
		 leftSensor = left->Get() ? 1 : 0;
		 middleSensor = middle->Get() ? 1 : 0;
		 rightSensor = right->Get() ? 1 : 0;

		 if (leftSensor == 0 && middleSensor == 1 && rightSensor == 1) {
		 myRobot->Drive(speed, -0.5); // right and middle sensors are on line	

		 } else if (leftSensor == 1 && middleSensor == 1 && rightSensor == 0) {
		 myRobot->Drive(speed, 0.5); // left and middle sensors are on line

		 } else if ((leftSensor == 1 && middleSensor == 1 && rightSensor
		 == 1) || (leftSensor == 1 && middleSensor == 0
		 && rightSensor == 1)) {
		 //CHECK - add support for 'y' line, if necessary
		 reachedEndOfLine = true;

		 } else if (leftSensor == 0 && middleSensor == 0 && rightSensor == 0) {
		 //robot is off the line, stop
		 myRobot->Drive(speed, 0.0);

		 } else if (leftSensor == 0 && middleSensor == 1 && rightSensor == 0) {
		 myRobot->Drive(speed, 0.0); //only middle sensor is on line, go forward
		 }

		 //raise the shoulder while following line, depending on which line robot is following
		 if (followingLineNumber == 1 || followingLineNumber == 3) {
		 //scoring on the highest, second column, peg
		 MoveShoulderTo(kTopRowSecondColumn);
		 releaseVoltage = kTopRowSecondColumn-0.1;
		 } else {
		 //scoring on the top row, first or third column, peg
		 MoveShoulderTo(kTopRow);
		 releaseVoltage = kTopRow-0.1;
		 }

		 }

		 //drop shoulder a bit, open gripper, and stop driving
		 while (VoltageReached(shoulderPotentiometerReading, releaseVoltage)
		 != 0) {
		 //at the end of the line, stop
		 myRobot->Drive(0.0, 0); // stop robot

		 gripperMotor->SetAngle(55); //opens gripper
		 MoveShoulderTo(releaseVoltage);
		 }

		 //reverse robot and lower shoulder
		 while (gameTimer->Get() < 14) {
		 myRobot->Drive(-speed, 0); // reverse robot
		 MoveShoulderTo(kPickUpPosition);
		 }
		 */
	}

	/*
	 * OPERATOR CONTROL using a arcade style drive
	 */
	void OperatorControl(void) {

		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl()) //This code will loop continuously as long it is operator control mode
		{
			GetWatchdog().Feed(); // Feed the watchdog	

			shoulderPotentiometerReading = (5-(shoulderPotentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1

			/* COMMENT GRIPPER CODE BELOW
			if (shoulderPotentiometerReading == 0) {
				gripperMotor->SetAngle(0); //opens gripper
			}
			else if (shoulderPotentiometerReading > 3) {
				gripperMotor->SetAngle(55); //closes gripper
			}
			*/

			if (leftstick->GetRawButton(4) == true) {
				shoulderMotor->Drive(-kMaxShoulderMotorSpeed, 0); //moves shoulderMotor down
				shoulderDestinationVoltage = shoulderPotentiometerReading;
			} else if (leftstick->GetRawButton(5) == true) {
				shoulderMotor->Drive(kMaxShoulderMotorSpeed, 0); //moves shoulderMotor up
				shoulderDestinationVoltage = shoulderPotentiometerReading;
			} else {
				
				//LEFTSTICK PRESETS
				if (leftstick->GetRawButton(2) == true) {
					//shoulderDestinationVoltage = kPickUpPosition;
				} else if (leftstick->GetRawButton(6) == true) {
					shoulderDestinationVoltage = kTopRow;
				} else if (leftstick->GetRawButton(7) == true) {
					shoulderDestinationVoltage = kMiddleRow;
				} else if (leftstick->GetRawButton(8) == true) {
					shoulderDestinationVoltage = kBottomRow;
				} else if (leftstick->GetRawButton(9) == true) {
					shoulderDestinationVoltage = kBottomRowSecondColumn;
				} else if (leftstick->GetRawButton(10) == true) {
					shoulderDestinationVoltage = kMiddleRowSecondColumn;
				} else if (leftstick->GetRawButton(11) == true) {
					shoulderDestinationVoltage = kTopRowSecondColumn;
				}

				MoveShoulderTo(shoulderDestinationVoltage);
			}
			
			
			
			//----------------------------------------------------------------
			

			//GRIPPER OPEN AND CLOSE
			if (leftstick->GetRawButton(1) == true) {
				gripperMotor->SetAngle(0); //opens gripper
			} else {
				gripperMotor->SetAngle(55); //closes gripper
			}
			
			//MINIBOT DEPLOYMENT 
			if (leftstick->GetRawButton(3) == true) {
				minibotDeployMotor->SetAngle(60); //releases four-bar
				myRobot->Drive(0.0, 0); // stop robot since controls are going to be inverted momentarily - don't want to go from full forward to full reverse instantly
			}

			//RIGHTSTICK PRESETS

			if (rightstick->GetRawButton(1) == true) {
				//back of the robot is moved forward by pushing forward on the joystick
				myRobot->ArcadeDrive((rightstick->GetY()),
						(rightstick->GetX()), false); // inverted drive control	
			} else {
				//front of the robot is moved forward by pushing forward on the joystick
				myRobot->ArcadeDrive(-(rightstick->GetY()),
						(rightstick->GetX()), false); // normal drive control		
			}
			
			//MINIBOT CLOSING/CLIMBING POLE
			if (rightstick->GetRawButton(3) == true) {
				//minibot closes only after its four-bar is dropped
				minibotCloseMotor1->SetAngle(45); //closes minibot so it starts climbing pole
				minibotCloseMotor2->SetAngle(45); //closes minibot so it starts climbing pole
			} else if (rightstick->GetRawButton(4) == true) {
				//MANUAL ARM CONTROL
				armMotor->Drive(-kMaxArmMotorSpeed/2.0, 0); //turn armMotor counter clockwise
			} else if (rightstick->GetRawButton(5) == true) {
				//MANUAL ARM CONTROL
				armMotor->Drive(kMaxArmMotorSpeed/1.5, 0); //turn armMotor cw
			} 
		}
	}

	void MoveShoulderTo(float shoulderDestinationVoltageToReach) {

		 if (VoltageReached(shoulderPotentiometerReading, shoulderDestinationVoltageToReach) == 1) {
			 //pot voltage is less than required, move shoulder up
			 shoulderMotor->Drive(kMaxShoulderMotorSpeed, 0);
		 } 
		 else if (VoltageReached(shoulderPotentiometerReading, shoulderDestinationVoltageToReach) == 2) {
			 //pot voltage is greater than required, move shoulder down
			 shoulderMotor->Drive(-kMaxShoulderMotorSpeed, 0);
		 } 
		 else {
			 //reached destination
			 shoulderMotor->Drive(0.0, 0.0);
			 shoulderDestinationVoltage = shoulderPotentiometerReading;
		 }
	}

	int VoltageReached(float voltage, float voltageToReach) {

		//0 = voltage has reached destination
		//1 = voltage is less than destination
		//2 = voltage is greater than destination
		//-1 = error

		if (voltage > voltageToReach-0.025 && voltage < voltageToReach+0.025) {
			return 0;
		} else if (voltage >= voltageToReach+0.025) {
			return 2;
		} else if (voltage <= voltageToReach-0.025) {
			return 1;
		} else {
			return -1;
		}

	}

};

START_ROBOT_CLASS(RobotDemo)
;
