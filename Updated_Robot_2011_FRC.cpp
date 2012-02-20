#include "WPILib.h"

/**
* This is a demo program showing the use of the RobotBase class.
* The SimpleRobot class is the base of a robot application that will automatically call your
* Autonomous and OperatorControl methods at the right time as controlled by the switches on
* the driver station or the field controls.
*/
//static float Vm = (ultraVal*1000);
//float Ri = (Vm/9.765625);


class RobotDemo : public SimpleRobot
{
   RobotDrive *myRobot; // robot drive system
   RobotDrive *JagBelt;
   
   Joystick *stick; // only joystick
   Jaguar *belt;
   Joystick *rightstick;
   RobotDrive *Drive;
   Jaguar *cim1;
   Jaguar *cim2;
   bool check, checkTwo,checkThree;
   Jaguar *pickStop;
   Relay *spike;
   Relay *window;
   DigitalInput *limitSwitch;
   DriverStationLCD *dsLCD;
   DriverStation *dash;  
   Servo *bar;
   Servo *bar2;
   

   AnalogChannel *gyro;
   AnalogChannel *Temperature;
   AnalogChannel *rangeFinder; // call for ultrasonic sensor
   
   bool precisionMode;
    ADXL345_I2C *accelerometer;
   
public:
   RobotDemo(void)
           // these must be initialized in the same order, ports 8 and nine are not being used
   {
       /*Initialize all the yummy delicious variables, they are so yummy and tasty! Gotta love em :D
        * hoho
        * hoohoh
        * hohoho
        * hehehe, love it
        */
       bar = new Servo (7);
       bar2= new Servo (8);
       stick = new Joystick(1);
       rightstick = new Joystick(2);
       //belt= new Jaguar(5);
       JagBelt= new RobotDrive(5,10);
       //Drive = new RobotDrive(2,1);
       myRobot = new RobotDrive(2,1);
       cim1 = new Jaguar(3);
       cim2= new Jaguar(6);    
       pickStop= new Jaguar(4);
       
       spike = new Relay(1);
       window= new Relay(2);
       limitSwitch = new DigitalInput(1);
       dash = DriverStation::GetInstance();
       dsLCD = DriverStationLCD::GetInstance();
       rangeFinder = new AnalogChannel(5);
       gyro = new AnalogChannel(6);
       Temperature = new AnalogChannel(7);
       accelerometer = new ADXL345_I2C (1,ADXL345_I2C::kRange_2G);
       //float Ri = (Vm/9.765625);            
       myRobot->SetExpiration(0.1);    
   }

   /**
    * Drive left & right motors for 2 seconds then stop
    */
   void Autonomous(void)
   {
       /*
       myRobot.SetSafetyEnabled(false);
       myRobot.Drive(0.5, 0.0);     // drive forwards half speed
       Wait(2.0);                 //    for 2 seconds
       myRobot.Drive(0.0, 0.0);     // stop robot
       */
   }

   /**
    * Runs the motors with arcade steering.
    */
   void OperatorControl(void)
   {
       myRobot->SetSafetyEnabled(true);
       while (IsOperatorControl())
       {
           bool setLimit;
           double cimValue1= scaleThrottle(-(stick->GetZ())); //Set desired speed from the Throttle, assuming from -1 to 1, also invert the cim, since we want it to rotate coutnerclockwise/clockwise
           double cimValue2= scaleThrottle((stick->GetZ())); //Set desired speed from the Throttle, assuming from -1 to 1
           //For shooter
           /*if (stick.GetRawButton(1) == true) {
                           //back of the robot is moved forward by pushing forward on the joystick
                           myRobot.ArcadeDrive((stick.GetY()),
                                   (stick.GetX()), false); // inverted drive control    
                       } else {
                           //front of the robot is moved forward by pushing forward on the joystick
                           myRobot.ArcadeDrive(-(stick.GetY()),
                                   (stick.GetX()), false); // normal drive control        
                       }
                       */
           //For manual button speed control, this sets the speed
            if (stick->GetRawButton(4) == true) {
                      cim1->Set(cimValue1); //use the value from the throttle to set cim speed
                      cim2->Set(cimValue2);//Get speed from throttle, and then scale it
                      setLimit = true;
                      
                       }
                       else {
                           cim1->Set(0.0);
                           cim2->Set(0.0);
                           setLimit = false;
                       }

                          
           
           //For precisebelt pickup
           if (stick->GetRawButton(1) == true) {
               //back of the robot is moved forward by pushing forward on the joystick
               if (setLimit == true) {
                   JagBelt->ArcadeDrive(0.1,
                       (stick->GetX()), false); // inverted drive control
               } else {
                   JagBelt->ArcadeDrive((stick->GetY()),
                       (stick->GetX()), false); // inverted drive control
               }
           } else {
               //front of the robot is moved forward by pushing forward on the joystick
               if (setLimit == true) {
                                   JagBelt->ArcadeDrive(-0.1,
                                       (stick->GetX()), false); // inverted drive control
                               } else {
                                   JagBelt->ArcadeDrive(-(stick->GetY()),
                                       (stick->GetX()), false); // inverted drive control
                               }        
           }
           //For normal belt pickup
           /*if (stick->GetRawButton(6) == true) {
                                       JagBelt->Drive(1.0, 0); //opens gripper
                                       
                                   } else {
                                       JagBelt->Drive(0.0, 0); //closes gripper
                                   }
                                   */
                       
                       
           
           //For drive
           
           if (rightstick->GetRawButton(1) == true) {
                           //back of the robot is moved forward by pushing forward on the joystick
                       if (rightstick->GetRawButton(10) == true) {
                           precisionMode= true;
                       }
                       else {
                           precisionMode= false;
                       }
                           myRobot->ArcadeDrive((rightstick->GetY()), (rightstick->GetX()), precisionMode); // inverted drive control    
                       } else
                       {
                           if (rightstick->GetRawButton(10) == true) {
                                                       precisionMode= true;
                                                   }
                                                   else {
                                                       precisionMode= false;
                                                   }
                           //front of the robot is moved forward by pushing forward on the joystick
                           myRobot->ArcadeDrive(-(rightstick->GetY()), (rightstick->GetX()), precisionMode); // normal drive control        
                       }
           
       
                               
           
           /*if (stick->GetRawButton(8) == true) {
                                               cim1->Set(-0.37); //run cim 1 at 50% speed counterclockwise??
                                               cim2->Set(0.37); // run cim  at 50% speed clockwise
                                               check = true; //indicate if motors are running
                                           } else if (check == true){ // if motors are running a
                                               Wait(2.0);
                                               belt->Set(1); // run the belt
                                               Wait(2.0); // one sec delay
                                               belt->Set(0.0); // turn belt off
                                               check = false; // put new check
                                           }
                                           else if (check == false){ //if false
                                                // 2 sec delay to wait for the first ball to shoot
                                               cim1->Set(0.0); //stop cims
                                               cim2->Set(0.0);
                                           }
                                           else { // Stop everything
                                               cim1->Set(0.0); //stop cims
                                               cim2->Set(0.0);
                                               belt->Set(0.0);
                                           }
                                           */

           
           
                                  
                               
           
           //Code for using window motor
           if (rightstick->GetRawButton(4)) {
               window->Set(Relay::kOn);                        
               window->Set(Relay::kForward); // tell window motor to go forward
                           
                       
                        
                       }  else if (rightstick->GetRawButton(5) == true){
                       window->Set(Relay::kOn);
                       window->Set(Relay::kReverse); //tell window motor to go backward


                       }
                       else {
                       //Wait(1.0);
                       window->Set(Relay::kOff); //turn it off, if the relays aint being used
                       }
           
           
           
           //Code for Banebot Motor for stopping ballz
           if (stick->GetRawButton(2) == true) { // press button TWO to close
                               pickStop->Set(-0.5); //closes ball stopper
                               Wait(1);
                               pickStop->Set(0.0);
                               } else if (stick->GetRawButton(3) == true){ //press button three to open
                                   pickStop->Set(0.5); //opens ball stopper
                                   Wait(1.2); // too slow, so needs more time
                                   pickStop->Set(0.0);
                               }
                               else if (stick->GetRawButton(5)== true){ //press 5 to stop imediately, useful for adjusting angles...
                                           //Wait(1.0);
                                   pickStop->Set(0.0);
                               }
                               
                               
//Code for ... servoooo
           if (stick->GetRawButton(10) == true) { //press 10 on the left stick...
           bar->SetAngle(60); // set the angles to 60...clockwise?
           bar2->SetAngle(60);
       } else if (stick->GetRawButton(11) == true) // press 11 on the left stick
       {
           bar->SetAngle(-60);   //set the angles to -60...counterclockwise?
          bar2->SetAngle(-60);
            }
            
           // Initialize functions...
           //  RelayServo();
           //PreciseBelt();
            //UltrasonicRange();
           // Accelerometer();
           
           //dash->GetPacketNumber();

                
// send data back to dashboard
           dash->GetPacketNumber(); //not sure why this is here 0_0
           //int limitValue= limitSwitch->Get() ? 1 : 0; // retrieve 1 and 0 only.../ look for 0 and 1
           float servoAngle = bar->GetAngle();
           //dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Limit State: %d", limitValue); //send data back to driver station...
          // dsLCD->Printf(DriverStationLCD::kUser_Line2, 2, "Servo Angle: %f", servoAngle); //send data back to driver station...
                   float gyroVal = gyro -> GetVoltage();//Gets voltage  from gyro
                   float ultraVal = rangeFinder -> GetVoltage(); //Get voltage from ultrasonic sensor
                   float tempVal = Temperature -> GetVoltage();//Gets temperature

//Do the math to convert data received from the ultrasonic volts->miliVolts->milivolts per inch->inches
                   float Vm = (ultraVal*1000);
                   float Ri = (Vm/9.765625);
                   
                   // Print data back to dashboard
                   dsLCD->Printf(DriverStationLCD::kUser_Line1, 1, "Ultrasonic Range: %f",Ri);
                   dsLCD->Printf(DriverStationLCD::kUser_Line2, 1, "Gyro: %f", gyroVal);
                   dsLCD->Printf(DriverStationLCD::kUser_Line3, 1, "Temperature: %f", tempVal);
                   dsLCD->Printf(DriverStationLCD::kUser_Line4, 1, "Cim1 Speed: %f%%", (cimValue1*100)); //display speed that the mototrs are reunning at different percentages...
                   dsLCD->Printf(DriverStationLCD::kUser_Line5, 1, "Cim2 Speed: %f%%", (cimValue2*100));
		   //Update dashboard
                   dsLCD->UpdateLCD();
                           
                   
       }
       
       }
   //Code for Ultrasonic Distance Tracking for Adusting speed of cim 1 and cim 2
void Accelerometer(void) {
   //float accelValueX= accelerometer->GetAcceleration(ADXL345_I2C::kAxis_X);
//    float accelValueY= accelerometer->GetAcceleration(ADXL345_I2C::kAxis_Y);
   //float accelValueZ= accelerometer->GetAcceleration(ADXL345_I2C::kAxis_Z);

   //printf("Axis X: %d  Axis Y %d Axis Z %d\n", accelValueX, accelValueY,accelValueZ);
   /*dsLCD->Printf(DriverStationLCD::kUser_Line4, 4, "Axis X: %f", accelValueX);
   dsLCD->Printf(DriverStationLCD::kUser_Line5, 5, "Axis Y: %f", accelValueY);
   dsLCD->Printf(DriverStationLCD::kUser_Line6, 6, "Axis Z: %f", accelValueZ);

    dsLCD->UpdateLCD(); // keep the dashlcd updated..*/
}
   
/*    void UltrasonicRange(void) {
       float ultraVal = rangeFinder -> GetVoltage(); //Get voltage from ultrasonic sensor
         float Vm = (ultraVal*1000);// math to convert to inches
         float Ri = (Vm/9.765625);// math to convert to to inches part two

   /*if (Ri == 120) { //press 10 on the left stick...
           bar->SetAngle(60); // set the angles to 60...clockwise?
           //bar2->SetAngle(60);
       } else if (stick.GetRawButton(11) == true) // press 11 on the left stick
       {
           bar->SetAngle(-60);   //set the angles to -60...counterclockwise?
         //  bar2->SetAngle(-60);
                */
   /*    dsLCD->Printf(DriverStationLCD::kUser_Line3, 3, "Ultrasonic Range: %f",Ri);
       dsLCD->UpdateLCD(); // keep the dashlcd updated...

   }*/

//    void RelayServo(void) {
       /*if (stick.GetRawButton(4) ==true) {
                       window->Set(Relay::kOn);                        
                       window->Set(Relay::kForward); // tell window motor to go forward
                       Wait(0.5);            
                       checkTwo=false;
                               }  else if (checkTwo == false){
                               bar->SetAngle(60);    
                              // window->Set(Relay::kOn);
                               //window->Set(Relay::kReverse); //tell window motor to go backward


                               }
                               else if (stick.GetRawButton(5) ==true  ) {
                               //Wait(1.0);
                                   window->Set(Relay::kOn);
                                   window->Set(Relay::kReverse); //tell window motor to go backward
                                   Wait(0.5);
                                   checkTwo = true;
                                }  else if (checkTwo == true){
                                  bar->SetAngle(-60);                               
                                                               }    
                                else {
                                window->Set(Relay::kOff); //turn it off, if the relays aint being used
                                bar->SetAngle(0);
                               }
                               */
       
   //    }
/*void PreciseBelt(void) {
   //For precisebelt pickup
               if (stick->GetRawButton(1) == true) {
                               belt->Set(0.4); //opens gripper
                               checkThree=false;
                           } else if (checkThree == false){
                               belt->Set(0); //closes gripper
                           }
                           
                           
}*/

double scaleThrottle(double x) {
	return (x + 1) / 2;
}

};

START_ROBOT_CLASS(RobotDemo);

