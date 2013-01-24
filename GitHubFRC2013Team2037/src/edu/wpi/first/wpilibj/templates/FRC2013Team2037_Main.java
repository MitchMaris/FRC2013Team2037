/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;



    /* --Xbox Controller buttons
     * 
     * &- means 0 to -1
     * &+ means 0 to 1
     * 
     * Left Stick = Axis 1,2 (1 is L- R+, 2 is U- D+)
     * Right Stick = Axis 4,5 (4 is L- R+, 5 is U- D+)
     * Left Paddle = Axis 3 +
     * Right Paddle = Axis 3 -
     * Left Bumper = Button 5
     * Right Bumper = Button 6
     * X = Button 3
     * Y = Button 4
     * A = Button 1
     * B = Button 2
     * Start = Button 8
     * Back = Button 7
     * D-Pad = Axis 6 (L- & R+ only)
     */



import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class FRC2013Team2037_Main extends SimpleRobot {
    
    
    Joystick m_xBox1 = new Joystick(1);  //driver joystick, controller 1
    Joystick m_xBox2 = new Joystick(2);  //shooter joystick, controller 2
    RobotDrive m_mecanumDrive = new RobotDrive(1,2,3,4);    
    double m_magnitude;
    double m_direction;
    double m_rotation;
    
    
    public void robotInit() {

    }

    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {
        while (isAutonomous() && isEnabled()) {
            Timer.delay(1);
        }
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            
            double m_deadZone = 0.133;  //we need to play with this number to see what needs to be changed
            double m_xb1_ax1;
            double m_xb1_ax2;
            double m_xb1_ax3;
            double m_xb1_ax4;
            double m_xb1_ax5;
            double m_xb1_ax6;
            
            //this is from the left joystick
            if (Math.abs(m_xBox1.getMagnitude()) > m_deadZone)
            {
                m_magnitude = m_xBox1.getMagnitude();
            }
            else
            {
                m_magnitude = 0;
            }
            //this is from the left joystick
            //leave in if we need to shape the number any
            if (Math.abs(m_xBox1.getDirectionDegrees()) > 0) //m_deadZone)
            {
                m_direction = m_xBox1.getDirectionDegrees();
            }
            else
            {
                m_direction = 0;
            }
            
            //this is from the right joystick
            if (Math.abs(m_xBox1.getX(GenericHID.Hand.kRight)) > m_deadZone)
            {
                m_rotation = m_xBox1.getX(GenericHID.Hand.kRight);
            }
            else
            {
                m_rotation = 0;
            }
           //debug code
            if (Math.abs(m_xBox1.getRawAxis(1)) > m_deadZone)
            {
                m_xb1_ax1 = m_xBox1.getRawAxis(1);
            }
            else
            {
                m_xb1_ax1 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(2)) > m_deadZone)
            {
                m_xb1_ax2 = m_xBox1.getRawAxis(2);
            }
            else
            {
                m_xb1_ax2 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(3)) > m_deadZone)
            {
                m_xb1_ax3 = m_xBox1.getRawAxis(3);
            }
            else
            {
                m_xb1_ax3 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(4)) > m_deadZone)
            {
                m_xb1_ax4 = m_xBox1.getRawAxis(4);
            }
            else
            {
                m_xb1_ax4 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(5)) > m_deadZone)
            {
                m_xb1_ax5 = m_xBox1.getRawAxis(5);
            }
            else
            {
                m_xb1_ax5 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(6)) > m_deadZone)
            {
                m_xb1_ax6 = m_xBox1.getRawAxis(6);
            }
            else
            {
                m_xb1_ax6 = 0;
            }


            if (m_magnitude != 0)
            {
                System.out.println("Magnitude: " + m_magnitude);
            }
            if (Math.abs(m_xb1_ax1) > 0 || Math.abs(m_xb1_ax2) > 0)
            {
                System.out.println("Direction: " + m_direction);
            }
            if (m_rotation != 0)
            {
                System.out.println("Rotation: " + m_rotation);
            }
            //debug code
            if (m_xb1_ax1 != 0)
            {
                System.out.println("Axis 1 = "+ m_xb1_ax1);
            }
            if (m_xb1_ax2 != 0)
            {
                System.out.println("Axis 2 = "+ m_xb1_ax2);
            }
            if (m_xb1_ax3 != 0)
            {
                System.out.println("Axis 3 = "+ m_xb1_ax3);
            }
            if (m_xb1_ax4 != 0)
            {
                System.out.println("Axis 4 = "+ m_xb1_ax4);
            }
            if (m_xb1_ax5 != 0)
            {
                System.out.println("Axis 5 = "+ m_xb1_ax5);
            }
            if (m_xb1_ax6 != 0)
            {
                System.out.println("Axis 6 = "+ m_xb1_ax6);
            }
                Timer.delay(1);
                
//             /* FPS controls for mecanum wheels */
//            magnitude = leftStick.getMagnitude();
//            direction = leftStick.getDirectionDegrees();
//            rotation = rightStick.getX();
                
            m_mecanumDrive.mecanumDrive_Polar(m_magnitude, m_direction, m_rotation);
        
        }
    
}
    public void test() {
    
    }
}
//                      Our BASE FTC Robot movement code. For reference. will remove once we test the Mecanum setup.
//
//
//
//                        #pragma config (Hubs ,  S1 , HTMotor ,  HTMotor ,  HTServo ,  HTServo) 
//                        #pragma config (Sensor , S2 ,     legoRBG ,        sensorCOLORBLUE) 
//                        #pragma config (Sensor , S3 ,     irSeekerSensor , sensorHiTechnicIRSeeker1200) 
//                        #pragma config (Sensor , S4 ,     compassSensor ,  sensorI2CHiTechnicCompass) 
//                        #pragma config (Motor ,  motorA ,           ,             tmotorNXT , PIDControl) 
//                        #pragma config (Motor ,  motorB ,           ,             tmotorNXT , PIDControl) 
//                        #pragma config (Motor ,  motorC ,           ,             tmotorNXT , PIDControl) 
//                        #pragma config (Motor ,  mtr_S1_C1_1 ,     frontRightMotor , tmotorTetrix , openLoop) 
//                        #pragma config (Motor ,  mtr_S1_C1_2 ,     backRightMotor , tmotorTetrix , openLoop) 
//                        #pragma config (Motor ,  mtr_S1_C2_1 ,     frontLeftMotor , tmotorTetrix , openLoop , reversed) 
//                        #pragma config (Motor ,  mtr_S1_C2_2 ,     backLeftMotor , tmotorTetrix , openLoop , reversed)
//
//
//                        #pragma debuggerWindows("joystickSimple"); 
//                        #include "JoystickDriver.c ";
//
//                        const int joystickDeadzoneValue = 10;  // deadstick value, is the amount of play in the joys
//                        //this function will take a joystick value, and a max motor value to make it easier within 
//
//                        int scaledMotorInput(int joyValue , int maxMotorValue) 
//                        {  
//                            //function variables  
//                            const float MAX_JOY_VAL = 128.0;    //the is the max value the joystick will output  
//                            int MAX_MOTOR_VAL;    //this is the max the motor input will take
//                              //check to see if a max motor val was passed to the function  
//                            if(maxMotorValue != 100)  
//                                {    
//                                    MAX_MOTOR_VAL = maxMotorValue;
//                                }    
//                            else  
//                                {    
//                                    MAX_MOTOR_VAL = 100;  
//                                }
//                            //calculate for scaled value  
//                            int direction = joyValue / abs(joyValue);     // 1 or -1 to determin forwards or backwards  
//                            float ratio = ((joyValue * joyValue) / (MAX_JOY_VAL * MAX_JOY_VAL));    //log math  
//                            int scaledVal = (ratio * MAX_MOTOR_VAL) * direction;    //scaled output math
//
//                            return scaledVal;     //returns the nice and neat value for motor driving 
//                        }
//
//
//                        //Kill code task 
//                        task killRobot() 
//                        {  
//                            while true)  
//                            {    
//                                if (joy1Btn(9) == 1)    
//                                {      
//                                    //zero the motors      
//                                    motor[frontLeftMotor] = 0;    //A      
//                                    motor[frontRightMotor] = 0;   //B      
//                                    motor[backRightMotor] = 0;    //C      
//                                    motor[backLeftMotor] = 0;     //D      
//                                    StopAllTasks();     //Stops all tasks currently running      
//                                    return;   //exits the current program.    
//                                }    
//                            wait1Msec(100);  
//                            }
//                        }
//
//                        // Main task for the robot 
//                        task main() 
//                        {
//
//                            //Outside main program loop, task start/stop code  
//                            StopTask(displayDiagnostics);  // disables the diag screen that's delivered from the joyst  
//                            StartTask(killRobot); //start the kill robot task
//
//                            //main variables  
//                            //main task variables used inside the main control loop  
//                            int maxMotorValBtn = 100;  //used to set a max value if desired, can also be changed dynam  
//                            float frontLeftMotorSpeed = 0; //set default motor speed variable    A  
//                            float frontRightMotorSpeed = 0; //set default motor speed variable   B  
//                            float backRightMotorSpeed = 0; //set default motor speed variable    C  
//                            float backLeftMotorSpeed = 0; //set default motor speed variable     D  
//                            float motorPowerValue = 0;  //set default power variable  
//                            float x1JoystickValue = 0; //set default x1 variable  
//                            float y1JoystickValue = 0; //set default y1 variable  
//                            float x2JoystickValue = 0; //set default x2 variable  
//                            string str1 = "frL ";  //used for displaying on the NXT  
//                            string str2 = "frR ";  //used for displaying on the NXT  
//                            string str3 = "baR ";  //used for displaying on the NXT  
//                            string str4 = "baL ";  //used for displaying on the NXT  
//                            string str5 = "pow ";  //used for displaying on the NXT  
//                            string str6 = "xAx ";  //used for displaying on the NXT  
//                            string str7 = "yAx ";  //used for displaying on the NXT
//
//                            // zero's both motors  
//                            motor [frontLeftMotor ] = 0;    //A  
//                            motor [frontRightMotor ] = 0;  //B  
//                            motor [backRightMotor ] = 0;    //C  
//                            motor [backLeftMotor ] = 0;      //D
//
//                            //clears the display.  
//                            bNxtLCDStatusDisplay = false ; //disables default screen  
//                            eraseDisplay ();
//
//
//                        //main control loop, no false condition. See abort button  
//                        while (true)
//                        {
//                            //poll the joysitck for current values    
//                            getJoystickSettings(joystick);
//                            //Create "deadzone" for Y1    
//                            if (abs(joystick.joy1_y1) >= joystickDeadzoneValue)
//                            {
//                                y1JoystickValue = scaledMotorInput(joystick.joy1_y1, maxMotorValBtn);
//                                y1JoystickValue = y1JoystickValue * 0.5;
//                            }
//                            else
//                            {
//                                y1JoystickValue = 0;
//                            }
//
//                            //Create "deadzone" for X1    
//                            if (abs(joystick.joy1_x1) >= joystickDeadzoneValue)
//                            {
//                                x1JoystickValue = scaledMotorInput(joystick.joy1_x1, maxMotorValBtn);
//                                x1JoystickValue = x1JoystickValue * 0.5 ;
//                            }
//                            else
//                            {
//                                x1JoystickValue = 0;
//                            }
//
//                            //Create "deadzone" for X2    
//                            if (abs (joystick.joy1_x2) >= joystickDeadzoneValue)
//                            {
//                              x2JoystickValue = scaledMotorInput(joystick.joy1_x2, maxMotorValBtn);
//                              x2JoystickValue = x2JoystickValue * 0.5;
//                            }
//                            else
//                            {
//                                x2JoystickValue = 0;
//                            }
//
//                            // code to set the max motor value    
//                            if (abs(x1JoystickValue) >= abs(y1JoystickValue))
//                            {
//                                motorPowerValue = x1JoystickValue;
//                            }
//                            else if (abs(x1JoystickValue) <= abs(y1JoystickValue ))
//                            {
//                                motorPowerValue = y1JoystickValue;
//                            }
//                            else
//                            {
//                                motorPowerValue = 0;
//                            }
//
//                            //assigning motor power number to an variable    
//                            frontLeftMotorSpeed = y1JoystickValue + x2JoystickValue + x1JoystickValue;
//                            frontRightMotorSpeed = y1JoystickValue - x2JoystickValue - x1JoystickValue;
//                            backRightMotorSpeed = y1JoystickValue - x2JoystickValue + x1JoystickValue;
//                            backLeftMotorSpeed = y1JoystickValue + x2JoystickValue - x1JoystickValue;
//
//                            //display some info on the screen, lots of it.      
//                            nxtDisplayTextLine (1, "%s=%f ", str1 , frontLeftMotorSpeed );
//                            nxtDisplayTextLine (2, "%s=%f ", str2 , frontRightMotorSpeed );
//                            nxtDisplayTextLine (3, "%s=%f ", str3 , backRightMotorSpeed );
//                            nxtDisplayTextLine (4, "%s=%f ", str4 , backLeftMotorSpeed );
//                            nxtDisplayTextLine (5, "%s=%f ", str5 , motorPowerValue );
//                            nxtDisplayTextLine (6, "%s=%d ", str6 , joystick.joy1_x1 );
//                            nxtDisplayTextLine (7, "%s=%d ", str7 , joystick.joy1_y1 );
//
//                            //power to motors    
//                            motor[frontLeftMotor] = frontLeftMotorSpeed;    //A    
//                            motor[frontRightMotor] = frontRightMotorSpeed;  //B    
//                            motor[backRightMotor] = backRightMotorSpeed;    //C    
//                            motor[backLeftMotor] = backLeftMotorSpeed;      //D
//
//
//                            } //end of control while loop  
//
//                            return ; //kills the program  
//
//                        } //end of task main
