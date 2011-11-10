#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotBase class.
 * The SimpleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 */

//defines the constants (variables that don't change)

//represent voltage outputted by the potentiometer at certain positions
#define kBottomRow ((float)(1.1))
#define kMiddleRow ((float)(2.59))
#define kTopRow ((float)(4.28))
#define kBottomRowSecondColumn ((float)(1.35))
#define kMiddleRowSecondColumn ((float)(2.86))
#define kTopRowSecondColumn ((float)(4.5))
#define kPackingPosition ((float) (0.48))

//miscellaneous constants
#define kMaxShoulderMotorSpeed ((float) 1.0)
#define kNumberOfJoysticks ((int) 3)
#define kGripperOpenCloseButton ((int) 1) //button used to open/close gripper
#define kAutonomousDistance ((float) 141) //in inches, 141

//potentiometer constants
#define kMaxPotentiometerVoltage ((float)4.5)
#define kMinPotentiometerVoltage ((float)0.45)

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
	AnalogChannel *potentiometerChannel;
	Encoder *wheelEncoder;

	DigitalInput *left; // used to get 0 or 1 from light sensors
	DigitalInput *middle;
	DigitalInput *right;
	
	Servo *gripperMotor; //used to control motors
	RobotDrive *shoulderMotor;

	//important variables declared here
	float potentiometerReading; //variable used to hold potentiometer reading
	int holdButton [kNumberOfJoysticks*11 + 1]; //since there are 11 buttons on each joystick; this array is used to hold a button during the method moveShoulderArmToHeight
	int joystickHolder [kNumberOfJoysticks + 1];
	double encoderReading;
	bool distanceHasBeenReached;
	
	//method variables below
	int buttonCounter;
	int counter;
	double releaseReading;

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
		potentiometerChannel = new AnalogChannel(1);
		
		//wheel encoder initialization
		wheelEncoder = new Encoder (4,5); 	//4th parameter: 0 means k1x, 1 means k2x, 2 means k4x 
		wheelEncoder->Start();
		wheelEncoder->Reset();
		wheelEncoder->SetDistancePerPulse(0.0359039085);
		
		left = new DigitalInput(1); // these DigitalInput instances are used to get 0 or 1 from light sensors
		middle = new DigitalInput(2); // connected to digital sidecar's channel 1, 2, and 3
		right = new DigitalInput(3);

		gripperMotor = new Servo(4); // servo motor is connected to PWM out channel 4
		shoulderMotor = new RobotDrive (3, 10); //drive using motor connected to PWM 3 and the unused PWM 10


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
		bool openGripper = false;
		double speed = 0;

		//run this loop for 15 seconds - DON'T CHANGE THIS - it will affect speed values
		while (lineFollowingTimer->Get() <= 15) {

			//set speed depending on time
			speed = 1-((lineFollowingTimer->Get())/10);
			
			//ensures speed does not go beyond max
			if (speed >= 0.25 || speed <= -0.25) {
				speed = 0.25;
			}
			
			//read distance wheel has traveled
			encoderReading = wheelEncoder->GetDistance();
			potentiometerReading = (5-(potentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1

			if (DistanceTraveled(kAutonomousDistance) == false) {
			
				// LINE FOLLOWING CODE - do this because we have not traveled enough distance yet
				leftSensor = left->Get() ? 1 : 0;
				middleSensor = middle->Get() ? 1 : 0;
				rightSensor = right->Get() ? 1 : 0;

				if (leftSensor == 0 && middleSensor == 1 && rightSensor == 1) {
					myRobot->Drive(speed, -0.5); // right and middle sensors are on line			
				} else if (leftSensor == 1 && middleSensor == 1 && rightSensor == 0) {
					myRobot->Drive(speed, 0.5); // left and middle sensors are on line	
				} else if ((leftSensor == 1 && middleSensor == 1 && rightSensor == 1) || (leftSensor == 1 && middleSensor == 0 && rightSensor == 1) || (leftSensor == 1 && middleSensor == 0 && rightSensor == 0)) {
					myRobot->Drive(speed, 0.9); // all on line / turn left if at the 'y'
				} else if (leftSensor == 0 && middleSensor == 0 && rightSensor == 0) {
					myRobot->Drive(0.0, 0); // robot is off the line, stop it	
				} else if (leftSensor == 0 && middleSensor == 1 && rightSensor == 0) {
					myRobot->Drive(speed, 0.0);	//only middle sensor is on line
				} 
				
			}
			else {
				//have traveled to the peg, stop then go to top row preset and open griper
				myRobot->Drive(0.0, 0);	

				//open gripper to release tube
				if ((potentiometerReading < (kMiddleRow + 0.025)) && (potentiometerReading > (kMiddleRow - 0.025)) || openGripper == true) {
					gripperMotor->SetAngle(65); 
					openGripper = true;
					
					if ((potentiometerReading < ((kMiddleRow-0.1) + 0.025)) && (potentiometerReading > ((kMiddleRow-0.1) - 0.025)) && openGripper == true) {
						MoveArmTo(kMiddleRow-0.1, 2, 1, false);	//since the method won't wait for a button to be pressed, buttons can be 0
					}
					
				}
				else {
					//move arm to top row second column
					MoveArmTo(kMiddleRow, 2, 1, false);	//since the method won't wait for a button to be pressed, buttons can be 0
				}
			}

		}
		myRobot->Drive(0.0, 0); // stop robot	
		
	}

	/*
	 * OPERATOR CONTROL using a tank style drive
	 */
	void OperatorControl(void) {
		GetWatchdog().SetEnabled(true);
		while (IsOperatorControl()) //This code will loop continuously as long it is operator control mode
		{
			GetWatchdog().Feed(); // Feed the watchdog		
			//myRobot->TankDrive(-(rightstick->GetY()), -(leftstick->GetY())); // drive with inverted tankstyle
			myRobot->ArcadeDrive(-(rightstick->GetY()), (rightstick->GetX())/2, false); // drive with arcade drive
			potentiometerReading = (5-(potentiometerChannel->GetVoltage())); // reads the potentiometer at channel 1
			encoderReading = wheelEncoder->GetDistance();
			
			//NOTES
			// Analog module must be in slot 1 of cRIO
			//
			//
			//----------------------------------------------------------------------------

			//GRIPPER OPEN AND CLOSE - 45 degrees opens it, 0 closes it
			if (leftstick->GetRawButton(kGripperOpenCloseButton) == true) { 
				gripperMotor->SetAngle(65); //opens gripper
			} 
			else {
				gripperMotor->SetAngle(0); //closes gripper
			}
			

			
			// POTENTIOMETER PRESETS
			// MoveArmTo (float potentiometerVoltage, int stickNumber (1 for left, 2 for right), int buttonNumber, bool waitForButtonToBePressed)

			//in case a wrong preset a chosen
			if (leftstick->GetRawButton(2) == true) {
				ReleaseAllButtonsOnJoystickAndStopShoulderMotor (1);
			}
			
			MoveArmTo(kPackingPosition, 1, 3, true);	//when button # 6 on left joystick is pressed, move arm to packing robot position
			MoveArmTo(kBottomRowSecondColumn, 1, 8, true);	//when button # 4 on left joystick is pressed, move arm to bottom row, 2nd column
			MoveArmTo(kMiddleRowSecondColumn, 1, 7, true);	//when button # 2 on left joystick is pressed, move arm to middle row, 2nd column
			MoveArmTo(kTopRowSecondColumn, 1, 6, true);	//when button # 5 on left joystick is pressed, move arm to top row, 2nd column
			MoveArmTo(kBottomRow, 1, 9, true);	//when button # 4 on left joystick is pressed, move arm to bottom row
			MoveArmTo(kMiddleRow, 1, 10, true);	//when button # 2 on left joystick is pressed, move arm to middle row
			MoveArmTo(kTopRow, 1, 11, true);	//when button # 5 on left joystick is pressed, move arm to top row

			if (rightstick->GetRawButton(3) == true) {
				//initiates release function (when driver thinks its okay to release the tube)
				ReleaseTubeOnPeg();
			}
			
			//MANUAL CONTROL - FOR GETTING POT. VOLTAGES
			if (leftstick->GetRawButton(4) == true) {
				shoulderMotor->Drive(-kMaxShoulderMotorSpeed,0);	//moves shoulderMotor down
			}
			else if (leftstick->GetRawButton(5) == true) {
				shoulderMotor->Drive(kMaxShoulderMotorSpeed,0);		//moves shoulderMotor up
			}
			
			
		}
	}
	
	bool DistanceTraveled (double distanceToReach) {
		
		//if (wheelEncoder->GetDirection() == 0) {
			//wheel is moving forward
			distanceToReach = -distanceToReach;

			//check if distance has been reached
			if (encoderReading <= distanceToReach) {
				distanceHasBeenReached = true;
			}
			else if (encoderReading > distanceToReach) {
				distanceHasBeenReached = false;
			}
		//}
		/*
		else {
			//wheel is moving backward

			//check if distance has been reached
			if (encoderReading >= distanceToReach) {
				distanceHasBeenReached = true;
			}
			else if (encoderReading < distanceToReach) {
				distanceHasBeenReached = false;
			}
		}*/
		
		return distanceHasBeenReached;
	}
	
	void MoveArmTo(float potentiometerVoltage, int stickNumber, int buttonNumber, bool waitForButtonToBePressed) {

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
			if (potentiometerReading < potentiometerVoltage-0.025) {
				shoulderMotor->Drive(kMaxShoulderMotorSpeed, 0);	//move shoulder down
				
				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = true;
				}
			} 
			else if (potentiometerReading > potentiometerVoltage+0.025) {
				shoulderMotor->Drive(-kMaxShoulderMotorSpeed, 0);	//move shoulder up

				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = true;
				}
			} 
			else {
				//reached the desired position
				shoulderMotor->Drive(0,0);							//stop shoulder
				
				//used to hold certain joystick buttons to repeat moving shoulder so button doesnt need to be held down
				if (waitForButtonToBePressed == true) {
					holdButton[buttonNumber] = false;
				}
			}
			
		}
	}
	void ReleaseTubeOnPeg () {
		if (counter == 0) {
			releaseReading = potentiometerReading-0.2;
			counter++;
		}
		else if (counter > 0 && (potentiometerReading < (releaseReading + 0.02)) && (potentiometerReading > (releaseReading - 0.02))) {
			shoulderMotor->Drive(-kMaxShoulderMotorSpeed,0);
		} 
		else {
			counter = 0;
			shoulderMotor->Drive(0,0);
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

};

START_ROBOT_CLASS(RobotDemo)
;

