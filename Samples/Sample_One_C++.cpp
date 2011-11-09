#include "WPILib.h"

class RobotDemo : public SimpleRobot
{
	RobotDrive 			*drive;			// robot drive base object
	
	Joystick 			*left_stick;
	Joystick			*right_stick;
	Joystick 			*left_armstick;
	Joystick 			*right_armstick;
	
	DigitalInput 		*left;			// digital inputs for line tracking sensors
	DigitalInput 		*middle;
	DigitalInput 		*right;
	DigitalInput		*toggleswitch;
	
	DigitalInput 		*armUpperLimit;	// create the limit switch inputs
	DigitalInput 		*armLowerLimit;
	
	DigitalInput 		*pressure_switch;
	
	Solenoid     		*grabber;
	Solenoid     		*kicker;
	Solenoid     		*puncher;
	Solenoid     		*licker;
	
    Relay               *a_relay;
    Relay               *another_relay;
    Relay               *the_last_relay;
    Relay               *compressor;
    
    Jaguar       		*spinner;
	Victor       		*digger;
	
	Servo				*arrow;
	
	Gyro         		*yaw_gyro;
	Ultrasonic   		*distance_sensor;
	
	DriverStation 		*ds;			// driver station object for getting selections
	DriverStationLCD 	*dsLCD;

	int                 time_to_send;
    int                 pressure_switch_value;
	bool                valid_distance;
	double              range_to_target;

public:
	
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// RobotDemo constructor:
	// This code creates instances of the objects
	//
	RobotDemo() 
	{
		drive = new RobotDrive(1, 2, 3, 4);
		drive->SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		drive->SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		drive->SetExpiration(0.1);						// Update the motors at least every 100ms.
		
		left_stick = new Joystick(1);               	// Logitech Joystick on USB 1
		right_stick = new Joystick(2);              	// Logitech Joystick on USB 2
		left_armstick = new Joystick(3);            	// Logitech Joystick on USB 3
		right_armstick = new Joystick(4);           	// Logitech Joystick on USB 4
		
		left = new DigitalInput(4,1);               	// cRio slot 4, digital input 1
		middle = new DigitalInput(4,2);             	// cRio slot 4, digital input 2
		right = new DigitalInput(4,3);              	// cRio slot 4, digital input 3
		toggleswitch = new DigitalInput(4,6);
		
		armUpperLimit = new DigitalInput(4,4);	    	// cRio slot 4, digital input 4
		armLowerLimit = new DigitalInput(4,5);      	// cRio slot 4, digital input 5

		pressure_switch = new DigitalInput(4,8);    	// cRio slot 4, digital input 8
		
		grabber = new Solenoid(8,1);               	    // cRio slot 4, solenoid driver 1
		kicker = new Solenoid(8,2);                 	// cRio slot 4, solenoid driver 2
		puncher = new Solenoid(8,3);                	// cRio slot 4, solenoid driver 3
		licker = new Solenoid(8,4);                 	// cRio slot 4, solenoid driver 4
		
	    a_relay = new Relay(4,1);                   	// cRio slot 4, spike relay driver 1
	    another_relay = new Relay(4,2);             	// cRio slot 4, spike relay driver 2
	    the_last_relay = new Relay(4,3);            	// cRio slot 4, spike relay driver 3
        compressor = new Relay(4,8);                	// cRio slot 4, spike relay driver 4
		
		spinner = new Jaguar(4,5);              	    // cRio slot 4, PWM 5, Jaguar type of H-bridge electronics speed control
		digger = new Victor(4,6);           	  		// cRio slot 4, PWM 5, Jaguar type of H-bridge electronics speed control
		
		arrow = new Servo(4,7);
		
		yaw_gyro = new Gyro(1,1);                   	// cRio slot 1, analog input 1, gyroscope

		distance_sensor = new Ultrasonic(4, 6, 4, 7); 	// PING (cRio module,slot) / ECHO (cRio module,slot) 

		ds = DriverStation::GetInstance();	                 
		dsLCD = DriverStationLCD::GetInstance();
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// this routine send data back to the DS (driver station), it is unlikely you will ever need to touch this
	//
	void sendIOPortData() 
	{
		Dashboard &dash = DriverStation::GetInstance()->GetLowPriorityDashboardPacker();
		dash.AddCluster();
		{
			dash.AddCluster();
			{ //analog modules 
				dash.AddCluster();
				{
					for (int i = 1; i <= 8; i++) {
						dash.AddFloat((float) AnalogModule::GetInstance(1)->GetAverageVoltage(i));
					}
				}
				dash.FinalizeCluster();
				dash.AddCluster();
				{
					for (int i = 1; i <= 8; i++) {
						dash.AddFloat((float) AnalogModule::GetInstance(2)->GetAverageVoltage(i));
					}
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();

			dash.AddCluster();
			{ //digital modules
				dash.AddCluster();
				{
					dash.AddCluster();
					{
						int module = 4;
						dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
						dash.AddU8(DigitalModule::GetInstance(module)->GetRelayReverse());
						dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
						dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIODirection());
						dash.AddCluster();
						{
							for (int i = 1; i <= 10; i++) {
								dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							}
						}
						dash.FinalizeCluster();
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();

				dash.AddCluster();
				{
					dash.AddCluster();
					{
						int module = 6;
						dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
						dash.AddU8(DigitalModule::GetInstance(module)->GetRelayForward());
						dash.AddU16((short)DigitalModule::GetInstance(module)->GetDIO());
						dash.AddU16(DigitalModule::GetInstance(module)->GetDIODirection());
						dash.AddCluster();
						{
							for (int i = 1; i <= 10; i++) {
								dash.AddU8((unsigned char) DigitalModule::GetInstance(module)->GetPWM(i));
							}
						}
						dash.FinalizeCluster();
					}
					dash.FinalizeCluster();
				}
				dash.FinalizeCluster();
			}
			dash.FinalizeCluster();
			
			// Can't read solenoids without an instance of the object, so we will steal one that is out there
			dash.AddU8(grabber->GetAll());
			// dash.AddU8(Solenoid.getAllFromDefaultModule());
		    // dash.AddU8(Solenoid::GetInstance(8)->getAll());
		    // dash.AddU8((char) 0);
		}
		dash.FinalizeCluster();
		dash.Finalize();
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// This function is called when the robot enters autonomous mode.
	// 
	void Autonomous() 
	{
		float angle;

		yaw_gyro->Reset();
		while (IsAutonomous())
		{
			pressure_switch_value = pressure_switch->Get();
            if (pressure_switch_value)
                compressor->Set(Relay::kOff);
            else 
            	compressor->Set(Relay::kForward);

            angle = yaw_gyro->GetAngle();			     // current heading (0 = target)
			drive->Drive(-0.5, -angle / 30.0);           // proportionally drive in a straight line
			
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			// we will send data to the driver station once every N passes through this while loop
			time_to_send++;
			if (time_to_send > 50)
			{
				time_to_send = 0;
				
				// print something to the DS LCD
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Kell Robotics : Auto");
				if (valid_distance)
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Range: ");
				else
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Out of Range");
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Spinner : %f");
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Digger  : %f");
	            if (pressure_switch_value)
					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "PS Closed");
	            else 
					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "PS Open");
                dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "The bottom of the ocean");
				dsLCD->UpdateLCD();
				
				// send all of our info to the dashboard
				sendIOPortData();
			}
		}
		drive->Drive(0.0, 0.0);                          // stop robot
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// This function is called when the robot enters operator control.
	//
	void OperatorControl() 
	{
		float ls_y, rs_y;
		float spinner_speed, digger_speed;
		float angle_value_or_some_crap;
		
		bool grabber_button;
		bool kicker_button;
		bool puncher_button;
		bool licker_button;
		bool toggle_switch_state;
		
		bool a_relay_button;
		bool another_relay_button;
		bool the_last_relay_button;
		
		// float angle;

		yaw_gyro->Reset();
		
		time_to_send = 0;
		drive->SetSafetyEnabled(true);
		
		while (IsOperatorControl())
		{
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			// read the joysticks and do the tank drive
			ls_y = left_stick->GetY();
			rs_y = right_stick->GetY();
			drive->TankDrive(ls_y, rs_y);

			
//            angle = yaw_gyro->GetAngle();			     // current heading (0 = target)
	//		drive->Drive(.5, -angle / 30.0);           // proportionally drive in a straight line
			
			
			// operate the grabber, read the joystick button and actuate a solenoid
			grabber_button = left_stick->GetRawButton(1);
			if (grabber_button)
			{
				grabber->Set(true);
			}
			else
				grabber->Set(false);
			
			// operate the kicker, read the joystick button and actuate a solenoid
			kicker_button = left_stick->GetRawButton(2);
			if (kicker_button)
				kicker->Set(true);
			else
				kicker->Set(false);
			
			// operate the puncher, read the joystick button and actuate a solenoid
			puncher_button = left_stick->GetRawButton(3);
			if (puncher_button)
				puncher->Set(true);
			else
				puncher->Set(false);
			
			// operate the licker, read the joystick button and actuate a solenoid
			licker_button = left_stick->GetRawButton(4);
			if (licker_button)
				licker->Set(true);
			else
				licker->Set(false);
			
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // operate a relay for some odd reason
            a_relay_button = right_stick->GetRawButton(1);
            if(a_relay_button) 
                a_relay->Set(Relay::kForward);
            else 
                a_relay->Set(Relay::kOff);
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // operate a relay for some odd reason
            another_relay_button = right_stick->GetRawButton(2);
            if(another_relay_button) 
            	another_relay->Set(Relay::kForward);
            else 
            	another_relay->Set(Relay::kOff);
            
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            // operate a relay for some odd reason
            the_last_relay_button = right_stick->GetRawButton(3);
            if(the_last_relay_button) 
            	the_last_relay->Set(Relay::kForward);
            else 
            	the_last_relay->Set(Relay::kOff);
            
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			// operate the compressor, read the pressure switch and turn the compressor on/off
			pressure_switch_value = pressure_switch->Get();
            if (pressure_switch_value)
                compressor->Set(Relay::kOff);
            else 
            	compressor->Set(Relay::kForward);
            /////////////////////////////////////////////////////////////////////////////////////////////////////////
            toggle_switch_state = toggleswitch->Get();
            
            
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			spinner_speed = left_stick->GetZ();
			spinner->SetSpeed(spinner_speed);
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			angle_value_or_some_crap = left_armstick->GetZ();
			arrow->Set(angle_value_or_some_crap);
			
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			digger_speed = right_stick->GetZ();
			digger->SetSpeed(digger_speed);
						
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			valid_distance = distance_sensor->IsRangeValid();
			if (valid_distance)
				range_to_target = distance_sensor->GetRangeInches();
			
			/////////////////////////////////////////////////////////////////////////////////////////////////////////
			// we will send data to the driver station once every N passes through this while loop
			time_to_send++;
			if (time_to_send > 50)
			{
				time_to_send = 0;
				
				// print something to the DS LCD
				dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Kell Robotics : Manual");
				if (valid_distance)
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Range: %f", range_to_target);
				else
					dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Out of Range");
				dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Spinner : %f", spinner_speed);
				dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Digger  : %f", digger_speed);
	            if (pressure_switch_value)
					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "PS Closed");
	            else 
					dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "PS Open"); 
	            
	            
	            if (toggle_switch_state)
	            	dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "TS Closed");
	            else 
	            	dsLCD->Printf(DriverStationLCD::kUser_Line6, 1, "TS Open"); 
	            
				dsLCD->UpdateLCD();
				
				// send all of our info to the dashboard
				sendIOPortData();
			}
			Wait(0.005);
		}
	}
};

START_ROBOT_CLASS(RobotDemo);

