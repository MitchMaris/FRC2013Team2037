/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package edu.wpi.first.wpilibj.templates;





import edu.wpi.first.wpilibj.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Team2037Robot2012 extends SimpleRobot {
    Gyro RoboGyro = new Gyro(1);
    AnalogChannel TempSensor = new AnalogChannel(2);
    Joystick Xbox = new Joystick(1);
    RobotDrive RoboDrive = new RobotDrive(1,2);
    SpeedController KickMotor = new Victor(3);
    SpeedController SpinMotor = new Jaguar(4);
    Relay SpikeRelay1 = new Relay(1);
    Relay SpikeRelay2 = new Relay(2);
    Servo ServoUD = new Servo(9);
    Servo ServoLR = new Servo(10);
    
    
    
    /**
     * Passes value between -1 and 1
     * 
     * PWM Ports
     * Port 1 = Right Drive Motor
     * Port 2 = Left Drive Motor 
     * Port 3 = Kick Motors 
     * Port 4 = Jaguar, SpinMotor
     * Port 9 = Camera Servo (L & R)
     * Port 10 = Camera Servo 2 ( U & D)
     * 
     * Relay Ports
     * Port 1 = Right Light
     * Port 2 = Left Light
     * 
     * Analog Port
     * Port 1 = Gyro
     * Port 2 = Temperature Sensor
     * 
     * 
     * 
     * --Xbox Controller buttons
     * Left Stick = Axis 1,2 (1 is LR, 2 is UD)
     * Right Stick = Axis 4,5 (4 is LR, 5 is UD)
     * Left Paddle = Axis 3 Up
     * Right Paddle = Axis 3 Down
     * Left Bumper = Button 5
     * Right Bumper = Button 6
     * X = Button 3
     * Y = Button 4
     * A = Button 1
     * B = Button 2
     * Start = Button 8
     * Back = Button 7
     * D-Pad = Axis 6 (L & R only)
     * 
     */
    
    
    
    
    /**
     * This function is called once each time the robot enters autonomous mode.
     */
    public void autonomous() {

        
//        Variables used                
        double change;
        double temperature;
        double gyroDataStart;
        double gyroDataCurrent;
//        reset the gryo to zero
        RoboGyro.reset();
        

        
//        Code to enable the Watchdog kill switch
        getWatchdog().setEnabled(true);
        getWatchdog().setExpiration(0.1);

        
        
//        ****************************************************************        
//        Control loop, used to ensure the bot is connected, functioning and safe
        while (true && isAutonomous() && isEnabled())
        {getWatchdog().feed();  
        
//        ****************************************************************       
//        Gyro code to get it human readable
        
        RoboGyro.setSensitivity(0.2);
        gyroDataCurrent = RoboGyro.getAngle();        
        
//        ****************************************************************       
//        Temperature Sensor code to get the temp
//        Get temperature offset from 77 F
        change = TempSensor.getVoltage() - 2.5;
//        convert to degrees F (1000 to bring out of millivolts / 5 per degree)
        change *= 200;
//        apply offset to reference temperature
        temperature = 77 + change;
        
        
        
//        ****************************************************************        
//          Turn on both spike lights to indicate the robot is in Autonomous mode
        SpikeRelay1.setDirection(Relay.Direction.kForward);
            SpikeRelay1.set(Relay.Value.kOn);
        
        SpikeRelay2.setDirection(Relay.Direction.kForward);
            SpikeRelay2.set(Relay.Value.kOn);
        
//        ****************************************************************        
//        Set the LCD to display it's current mode
        String blankStr = "                             ";
        
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, "Autonomous ON:");
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, "Gyro Data "+ gyroDataCurrent);  //Math.floor(gyroDataCurrent));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, "temp is "+ Math.floor(temperature));
        DriverStationLCD.getInstance().updateLCD();
        
         Timer.delay(0.005);        
        
        }
//        Zero our the spike relays outside the loop
        SpikeRelay1.set(Relay.Value.kOff);
        SpikeRelay2.set(Relay.Value.kOff);
        
        String blankStr = "                             ";
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, "Autunomous OFF:");
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, blankStr);
        DriverStationLCD.getInstance().updateLCD();
        
    }

    
    /**
     * This function is called once each time the robot enters operator control.
     */
    
    
    public void operatorControl() {
        
//        Variables used                
        double change;
        double temperature;
        double gyroDataStart = RoboGyro.getAngle();
        double gyroDataCurrent;
        boolean driveMode = true;
        boolean controlEnabled = false;
        String driveModeStr = "Arcade Mode";
//        reset the gryo to zero
        RoboGyro.reset();
        


        
        
                
//        Code to enable the Watchdog kill switch
        getWatchdog().setEnabled(true);
        getWatchdog().setExpiration(0.1);

        
        
//        ****************************************************************        
//        Control loop, used to ensure the bot is connected, functioning and safe
        while (true && isOperatorControl()  && isEnabled())
        {getWatchdog().feed();

        
//        ****************************************************************        
//        xbox controller code inserted here 
           
        
//        DEBUGGING CODE
//        Uncomment to see a specific Axis, button or Accelerometer data or information
//        
//        System.out.println("Axis 1 = "+Xbox.getRawAxis(1));
//        System.out.println("Axis 2 = "+Xbox.getRawAxis(2));
//        System.out.println("Axis 3 = "+Xbox.getRawAxis(3));
//        System.out.println("Axis 4 = "+Xbox.getRawAxis(4));
//        System.out.println("Axis 5 = "+Xbox.getRawAxis(5));
//        System.out.println("Axis 6 = "+Xbox.getRawAxis(6));
//        if (Xbox.getRawButton(1))System.out.println("Button 1 ");
//        if (Xbox.getRawButton(2))System.out.println("Button 2 ");
//        if (Xbox.getRawButton(3))System.out.println("Button 3 ");
//        if (Xbox.getRawButton(4))System.out.println("Button 4 ");
//        if (Xbox.getRawButton(5))System.out.println("Button 5 ");
//        if (Xbox.getRawButton(6))System.out.println("Button 6 ");
//        if (Xbox.getRawButton(7))System.out.println("Button 7 ");
//        if (Xbox.getRawButton(8))System.out.println("Button 8 ");
//        if (Xbox.getRawButton(9))System.out.println("Button 9 ");
//        if (Xbox.getRawButton(10))System.out.println("Button 10 ");
//        System.out.println("LR "+ServoLR.getPosition());
//        System.out.println("UD "+ServoUD.getPosition());
        
        
        
//        ****************************************************************    
//        Enable and disable the robot via button presses
        if (Xbox.getRawButton(8))
        {
            controlEnabled = true;
        }
        
        if (Xbox.getRawButton(7))
        {
            controlEnabled = false;
//            Set everything to off/default as a saftey measure.
            SpikeRelay1.set(Relay.Value.kOff);
            SpikeRelay2.set(Relay.Value.kOff);
            ServoLR.set(0.5);
            ServoUD.set(0.0);
            SpinMotor.set(0);
            KickMotor.set(0);
            RoboDrive.stopMotor();
        }
        
        
        
        
//        ****************************************************************       
//        Temperature Sensor code to get the temp
//        Get temperature offset from 77 F
        change = TempSensor.getVoltage() - 2.5;
//        convert to degrees F (1000 to bring out of millivolts / 5 per degree)
        change *= 200;
//        apply offset to reference temperature
        temperature = 77 + change;
        
        
        
//        ****************************************************************       
//        Gyro code to get it human readable
        RoboGyro.setSensitivity(0.2);
        gyroDataCurrent = RoboGyro.getAngle();
        
        
        
        
//        ****************************************************************        
//        Enable the robot to use arcadeDrive mode via Xbox controller to drive it.
//        
//        driveMode = true means it's in Arcade Mode, this is the default
//        driveMode = false means it's in Tanks Mode.
////        
//        If Rotating the eye via ServoRL and ServoUD
//        Servo takes values from 0 to 1. Xbox sends values -1 to 1. Dividing by 1 matches controls. 
//        Adding 0.5 centers the LR servo
//        code below is to just control the video camera
//        
//        ServoLR.setPosition((Xbox.getRawAxis(4)/1) + 0.5);
//        ServoUD.setPosition(Xbox.getRawAxis(5)/1);

         if (controlEnabled == true && Xbox.getRawButton(6))
         {
             driveMode = false;
         }
         
         if (controlEnabled == true && Xbox.getRawButton(5))
         {
             driveMode = true;
         }
        
         
//         Control if condition to tell if the rebot has been enabled.
        if (controlEnabled == true)
        {
            
//            Arcade Mode and Tank Mode selection code, Arcade Mode is default
            if (Xbox.getRawButton(6))
            {
                driveMode = false;
            }

            if (Xbox.getRawButton(5))
            {
                driveMode = true;
            }     
            
//        Check to see if variable driveMode is true, then run the code below. This is the default mode. Arcade is default
            if (driveMode == true)
            {
                RoboDrive.arcadeDrive(Xbox,true);
                ServoLR.setPosition((Xbox.getRawAxis(4)/1) + 0.5);
                ServoUD.setPosition(Xbox.getRawAxis(5)/1);
                driveModeStr = "Arcade Mode";
                SpikeRelay1.setDirection(Relay.Direction.kForward);
                SpikeRelay1.set(Relay.Value.kOn);
                SpikeRelay2.set(Relay.Value.kOff);
            }
        
            
//        Check to see if TankMode has been enabled, then run code below.
            if (driveMode == false)
            {
                RoboDrive.tankDrive(Xbox.getRawAxis(2), Xbox.getRawAxis(5));
                ServoLR.set(0.5);
                ServoUD.set(0.0);
                driveModeStr = "Tank Mode";
                SpikeRelay2.setDirection(Relay.Direction.kForward);
                SpikeRelay2.set(Relay.Value.kOn);
                SpikeRelay1.set(Relay.Value.kOff);
            }
        
        
             
//        ****************************************************************        
//        Enable the robot to use the Xbox Controller's kickers to trigger action
            KickMotor.set(Xbox.getRawAxis(3));
                
                


        
        
//        ****************************************************************        
//        Enable the Jaguar speedController temp use.
            SpinMotor.set(Xbox.getRawAxis(6));
        
        
        
        
        
//        ****************************************************************        
//        Gyro code, button/gyro combo goes here.
            if (Xbox.getRawButton(4))
            {
                RoboGyro.reset();
                gyroDataStart = (RoboGyro.getAngle());
            }

            if (Xbox.getRawButton(2))
            {
                RoboGyro.reset();
                gyroDataStart = (RoboGyro.getAngle());
                while (gyroDataCurrent < (gyroDataStart + 5.5) || (!Xbox.getRawButton(5)))
                {
                    getWatchdog().feed();
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, "Gyro Start "+ gyroDataStart);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, "Gyro Curnt "+ gyroDataCurrent);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, "Temp is "+ Math.floor(temperature));
                    DriverStationLCD.getInstance().updateLCD();
//                    RoboDrive.arcadeDrive(0, 0.6);
                    RoboDrive.tankDrive(-0.8, 0.8);
                }
            }

            if (Xbox.getRawButton(1))
            {
                RoboGyro.reset();
                gyroDataStart = (RoboGyro.getAngle());
                while (gyroDataCurrent < (gyroDataStart + 11) || (!Xbox.getRawButton(5)))
                {
                    getWatchdog().feed();
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, "Gyro Start "+ gyroDataStart);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, "Gyro Curnt "+ gyroDataCurrent);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, "Temp is "+ Math.floor(temperature));
                    DriverStationLCD.getInstance().updateLCD();
//                    RoboDrive.arcadeDrive(0, 0.6);
                    RoboDrive.tankDrive(-0.8, 0.8);

                }
            }

            if (Xbox.getRawButton(3))
            {
                RoboGyro.reset();
                gyroDataStart = (RoboGyro.getAngle());
                while (gyroDataCurrent < (gyroDataStart + 16.5) || (!Xbox.getRawButton(5)))
                {
                    getWatchdog().feed();
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, "Gyro Start "+ gyroDataStart);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, "Gyro Curnt "+ gyroDataCurrent);
                    DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, "Temp is "+ Math.floor(temperature));
                    DriverStationLCD.getInstance().updateLCD();
//                    RoboDrive.arcadeDrive(0, 0.6);
                    RoboDrive.tankDrive(-0.8, 0.8);

                }
            }
//        End of Enable if statement
        }    
        

      

        
//        ****************************************************************        
//        Exampe code to update the DriverStationLCD with a string
        String isActiveStr;
        String blankStr = "                             ";
        if (controlEnabled == false)
        {
            isActiveStr = "DEACTIVATED";
        }
        else
        {
            isActiveStr = "ACTIVATED";
        }
//        Actual Driver Station display code
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, "Teleoperated ON:");
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, isActiveStr + " " + driveModeStr);
//        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, "Robot is "+ isActiveStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, blankStr);
//        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, driveModeStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, "Gyro Start "+ gyroDataStart);  //Math.floor(gyroDataStart)););
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, "Gyro Curnt "+ gyroDataCurrent);  //Math.floor(gyroDataCurrent));
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, "Temp is "+ Math.floor(temperature));
        DriverStationLCD.getInstance().updateLCD();
        
        
//        End Xbox Controller Code
//        Part of the watchdog loop, this is the amount of time in seconds to delay before 
//        running the loop again. Change as needed depending on the amount of code running. 
//        If the robot shows signs is laggin via xbox inputs, slightly increment.
         Timer.delay(0.005);
                 

        }
        
        //        Zero our the spike relays outside the loop
        SpikeRelay1.set(Relay.Value.kOff);
        SpikeRelay2.set(Relay.Value.kOff);
        
        String blankStr = "                             ";
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, "Teleoperated OFF:");
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, blankStr);
        DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, blankStr);
        DriverStationLCD.getInstance().updateLCD();
    }
    
}

