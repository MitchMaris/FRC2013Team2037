/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


//Version
// 0.1.5


package edu.wpi.first.wpilibj.templates;



    /* Current plug cofig
     * 
     * -- PWM
     * 1. Front Left
     * 2. Front Right
     * 3. Rear Right
     * 4. Rear Left
     * 
     * -- Spike
     * 1. m_spikeRelay1
     * 2. m_spikeRelay2  shooter angle
     * 
     * -- Digital
     * 1. m_microSwitch
     * 
     * -- Analog
     * 1. m_gyro
     * 
     */


    /* --Xbox Controller buttons
     * 
     *  #- means 0 to -1
     *  #+ means 0 to 1
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




import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStationLCD;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.camera.AxisCameraException;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.LinearAverages;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class FRC2013Team2037_Main extends SimpleRobot {
    
    //Robot Init variables.
    //Joysticks
    Joystick m_xBox1 = new Joystick(1);  //driver joystick, controller 1
    Joystick m_xBox2 = new Joystick(2);  //shooter joystick, controller 2
    
    //PWM
    //robot Drive init, motor input order is frontLeft, rearLeft, frontRight, rearRight
    //robot Drive actual layout is frontLeft, frontRight, rearRight, rearLeft
    RobotDrive m_mecanumDrive = new RobotDrive(1,4,2,3);
    
    SpeedController m_shooterDriveFront = new Victor(5); //Konnor //Drive wheels for shooter 
    SpeedController m_shooterDriveBack = new Jaguar(6);  //change if using Victor
    
    Servo m_servoLClutch = new Servo(7); 
    Servo m_servoRClutch = new Servo(8);
    Servo m_servoVisionLR = new Servo(9);
    
    //Digital
    DigitalInput m_microSwitch1 = new DigitalInput(1);  //microSwitch1
    DigitalInput m_microSwitch2 = new DigitalInput(2); //microSwitch2
    
    //Analog
    Gyro m_gyro = new Gyro(1);
    
    //Relays
    Relay m_spikeShooterAngle = new Relay(1); //Konnor //Spike relay for controling angle motors
    Relay m_spikeBatteryMotor = new Relay(2);
    Relay m_spikeFrisPusherMotor = new Relay(3);
    Relay m_spikeVisionGreenLED = new Relay(4);
    Relay m_spikeBlueLED = new Relay(5);
    Relay m_spikeRedLED = new Relay(6);
    Relay m_spikeGreenLED = new Relay(7);
    Relay m_spikeRelay = new Relay(8);  //spikeRelay to blink a light via microSwitch1  //can be removed once we use it.
    
    //Vision
    AxisCamera m_camera;          // the axis camera object (connected to the switch)
    CriteriaCollection m_cc;      // the criteria for doing the particle filter operation
    //END Robot Init variables.
    
    //Global Variables
    //motor power variables
    double m_magnitude;
    double m_direction;
    double m_rotation;
    double m_shooterFrontSpeed;
    double m_shooterBackSpeed;
    double m_autonomousLoopDelay = 0.075;
    double m_operatorControlLoopDelay = 0.075;
    
    //Target Variables
    
    double m_distanceCenterTarget = 1000000;
    boolean visableCenterTarget = false;
    int ageCenterTarget = 0;
    
    //Average Variables
    double m_tempVariable = 1000000;
    double m_average = 0;
    
    //gyro and temp Variables
    //double m_change;      for temp    
    //double m_temperature; for temp
    double m_gyroDataStart;
    double m_gyroDataCurrent;
    double m_gyroSensitivity = 0.20;
    
    //vision Variables
    double m_visionLoopDelay = 0.25;
    
    //LED Variables
    boolean m_blink = false;
    boolean m_lightOn = false;
    int m_blinkCounter = 0;
    
    //Tracking Variables
    double m_rotationCenterHigh;
    double m_rotationCenterMiddle;
    boolean m_highTargetVisible = false;
    boolean m_middleTargetVisible = false;
    
    //Screen Variables
    String m_blankScreenStr = "                             ";
    String m_screenLine1 = m_blankScreenStr;
    String m_screenLine2 = m_blankScreenStr;
    String m_screenLine3 = m_blankScreenStr;
    String m_screenLine4 = m_blankScreenStr;
    String m_screenLine5 = m_blankScreenStr;
    String m_screenLine6 = m_blankScreenStr;
    double m_screenLoopDelay = 0.5;
     
    //END Global Variables
    
    //State Variables
    int m_disabledCount = 0;
    //END State Variables
    
    //vision code Variables
    final int XMAXSIZE = 24;
    final int XMINSIZE = 24;
    final int YMAXSIZE = 24;
    final int YMINSIZE = 48;
    final double xMax[] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
    final double xMin[] = {.4, .6, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, .1, 0.6, 0};
    final double yMax[] = {1, 1, 1, 1, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, .5, 1, 1, 1, 1};
    final double yMin[] = {.4, .6, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05, .05,
								.05, .05, .6, 0};
    
    final int RECTANGULARITY_LIMIT = 60;
    final int ASPECT_RATIO_LIMIT = 75;
    final int X_EDGE_LIMIT = 40;
    final int Y_EDGE_LIMIT = 60;
    
    final int X_IMAGE_RES = 320;          //X Image resolution in pixels, should be 160, 320 or 640
    final double VIEW_ANGLE = 43.5;       //Axis 206 camera
    //END vision code Variables
    
    // SCORE!!! for vision
    public class Scores {
        double rectangularity;
        double aspectRatioInner;
        double aspectRatioOuter;
        double xEdge;
        double yEdge;
    }
    
    //This function is called once when the robot is powered on.
    public void robotInit() { 
        
        
        //start updateScreenTask task
        try {
            Thread screenThread = new Thread(updateScreenTask);
            screenThread.start();
            clearScreen();
            updateScreen(1, "robotInit()");
            updateScreen(2, "screen, CHECK........");
            System.out.println("screenThread init complete....");
        }
        catch (Exception ex) {
            System.out.println("screenThread init FAILED!!!!  Reason: " + ex);
        
        }
        
        m_camera = AxisCamera.getInstance();  // get an instance of the camera
        m_cc = new CriteriaCollection();      // create the criteria for the particle filter
        m_cc.addCriteria(NIVision.MeasurementType.IMAQ_MT_AREA, 500, 65535, false);
        
        m_spikeRelay.set(Relay.Value.kOff);
        m_spikeShooterAngle.set(Relay.Value.kOff);
        m_spikeBatteryMotor.set(Relay.Value.kOff);
        m_spikeFrisPusherMotor.set(Relay.Value.kOff);
        m_spikeVisionGreenLED.set(Relay.Value.kOff);
        m_spikeBlueLED.set(Relay.Value.kOff);
        m_spikeRedLED.set(Relay.Value.kOff);
        m_spikeGreenLED.set(Relay.Value.kOff);
        
        m_servoLClutch.set(0);
        m_servoRClutch.set(0.5);
        
        
        try {
            m_gyro.reset();
            Timer.delay(.5);
            updateScreen(3, "Gyro, GOOD........");
            System.out.println("Gyro init complete....");
        } 
        catch(Exception ex) {
            updateScreen(3, "Gyro, FAILED!!!!!!!!");
            System.out.println("Gyro init FAILED!!!!");
        }
        
        //start the vision processing task
        try {
            
            Thread visionThread = new Thread(visionProcessingTask);
            visionThread.start();
            updateScreen(4, "vision, CHECK........");
            System.out.println("visionProcessingTask init complete....");
        }
        catch (Exception ex) {
            updateScreen(4, "vision, FAILED!!!!!!!!");
            System.out.println("visionProcessingTask init FAILED!!!!  Reason: " + ex);
        }
                
        updateScreen(5, "abcdefghijklmnopqrstuvwxyz1234567891011121314151617181920");

        
    }

    
    
    //This function is called once each time the robot enters autonomous mode.
    public void autonomous() {
        clearScreen();
        updateScreen(1, "autonomous()");
        
        m_mecanumDrive.setSafetyEnabled(false);
        m_spikeRelay.set(Relay.Value.kForward);
        System.out.println("We are running!");
        int loopCount = 1;
        
        while (isAutonomous() && isEnabled()) {
            
            Timer.delay(m_autonomousLoopDelay);
            if (loopCount % 100 == 0 || loopCount == 1) {
                System.out.println("autonomous Loop " + loopCount);
            }
            loopCount++;
        }
        
        stopRobot();  //Kill the motors on the robot
    }

    //This function is called once each time the robot enters operator control.
    public void operatorControl() {
       
        clearScreen();
        updateScreen(1, "operatorControl()");
       
        //local Teleop variables
        double m_xb1DeadZone = 0.165;  //we need to play with this number to see what needs to be changed
        double m_xb1_ax1;
        double m_xb1_ax2;
        double m_xb1_ax3;
        double m_xb1_ax4;
        double m_xb1_ax5;
        double m_xb1_ax6;
        double m_slowMotorSpeed;
        int loopCount = 1;

        m_mecanumDrive.setSafetyEnabled(true);
        m_mecanumDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        m_mecanumDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        
        while (isOperatorControl() && isEnabled()) {
            m_slowMotorSpeed = .4;
            
            m_gyro.setSensitivity(m_gyroSensitivity);  //use to slow the number down. 360 rotation equals (fill in with correct number)
                        
            //reads the current gyro data
            m_gyroDataCurrent = m_gyro.getAngle();
            
            //kill button code
            if (m_xBox1.getRawButton(8) || m_xBox2.getRawButton(8)) {
                stopRobot();
            }
            
            //turbo button, may delete if not needed
            
            if(m_xBox1.getRawAxis(3) < -0.5) {
                m_slowMotorSpeed = 1;
            }
            
            //quick test to see if you have the driver controller
            if (m_xBox1.getRawButton(7)) { //I want the input from the D pad for all of these .getRawAxis
                updateScreen(6, "Driver Controller");
            }
            else {
                updateScreen(6, m_blankScreenStr);
            }
               
            //this is from the left joystick
            //takes the greatest number from any direction and sets it to the current speed
            if (Math.abs(m_xBox1.getMagnitude()) > m_xb1DeadZone)
            {
                m_magnitude = scaledInput(m_xBox1.getMagnitude());
                m_magnitude = m_magnitude * m_slowMotorSpeed;  //add if we need to slow her down.
            }
            else
            {
                m_magnitude = 0;
            }
            //this is from the left joystick
            //leave in if we need to shape the number any
            //top is 0, bottom is 180, then goes from -179 back around to -1
            if (Math.abs(m_xBox1.getDirectionDegrees()) > 0) //m_deadZone)
            {
                m_direction = m_xBox1.getDirectionDegrees();
                
            } 
            else
            {
                m_direction = 0;
            }

            //this is from the right joystick
            //right and left, to make the robot spin
            if (Math.abs(m_xBox1.getX(GenericHID.Hand.kRight)) > m_xb1DeadZone)
            {
                m_rotation = scaledInput(m_xBox1.getRawAxis(4));
                m_rotation = m_rotation * m_slowMotorSpeed;  //add if we need to slow her down.
            }
            else
            {
                m_rotation = 0;
            }
           
            
//            //remove below if we do not use.
//            //level all joystick inputs, run through scaling code
            if (Math.abs(m_xBox1.getRawAxis(1)) > m_xb1DeadZone)
            {
                m_xb1_ax1 = scaledInput(m_xBox1.getRawAxis(1));
            }
            else
            {
                m_xb1_ax1 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(2)) > m_xb1DeadZone)
            {
                m_xb1_ax2 = scaledInput(m_xBox1.getRawAxis(2));
            }
            else
            {
                m_xb1_ax2 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(3)) > m_xb1DeadZone)
            {
                m_xb1_ax3 = scaledInput(m_xBox1.getRawAxis(3));
            }
            else
            {
                m_xb1_ax3 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(4)) > m_xb1DeadZone)
            {
                m_xb1_ax4 = scaledInput(m_xBox1.getRawAxis(4));
            }
            else
            {
                m_xb1_ax4 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(5)) > m_xb1DeadZone)
            {
                m_xb1_ax5 = scaledInput(m_xBox1.getRawAxis(5));
            }
            else
            {
                m_xb1_ax5 = 0;
            }

            if (Math.abs(m_xBox1.getRawAxis(6)) > m_xb1DeadZone)
            {
                m_xb1_ax6 = scaledInput(m_xBox1.getRawAxis(6));
            }
            else
            {
                m_xb1_ax6 = 0;
            }
            //end remove 
            
            
            //debug code
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
//            if (m_xb1_ax1 != 0)
//            {
//                System.out.println("Axis 1 = "+ m_xb1_ax1);
//            }
//            if (m_xb1_ax2 != 0)
//            {
//                System.out.println("Axis 2 = "+ m_xb1_ax2);
//            }
//            if (m_xb1_ax3 != 0)
//            {
//                System.out.println("Axis 3 = "+ m_xb1_ax3);
//            }
//            if (m_xb1_ax4 != 0)
//            {
//                System.out.println("Axis 4 = "+ m_xb1_ax4);
//            }
//            if (m_xb1_ax5 != 0)
//            {
//                System.out.println("Axis 5 = "+ m_xb1_ax5);
//            }
//            if (m_xb1_ax6 != 0)
//            {
//                System.out.println("Axis 6 = "+ m_xb1_ax6);
//            }
            
            //Uncomment to see more debug lines
            //System.out.println("MicroSwitch1 says.... " + m_microSwitch1.get());
            //System.out.println("MicroSwitch2 says.... " + m_microSwitch2.get());
//            System.out.println("SpikeRelay says....  " + m_spikeRelay.get());
//            System.out.println("Gyro Says....        " + m_gyroDataCurrent);
            
            
            //microSwitch and spikeRelay testing code
            if (m_microSwitch1.get() == false){
                m_spikeRelay.set(Relay.Value.kOff);
            }
            else {
                m_spikeRelay.set(Relay.Value.kForward);
            }
            
            // Konnor added this
            if (m_xBox1.getRawAxis(6) == 1) {
                m_spikeShooterAngle.set(Relay.Value.kForward);
            }
            else if (m_xBox1.getRawAxis(6) == -1) {
                m_spikeShooterAngle.set(Relay.Value.kReverse);
            }
            else if (m_xBox1.getRawAxis(6) == 0){
                m_spikeShooterAngle.set(Relay.Value.kOff);
            }
            
            
//            if (m_xBox1.getRawButton(4)) {
//                m_shooterFrontSpeed = 0;
//                m_shooterBackSpeed = 0;
//            }
//            else if (m_xBox1.getRawButton(1)) {
//                m_shooterFrontSpeed = 1;
//                m_shooterBackSpeed = 1;
//            }
//            
//            
//            if (m_xBox1.getRawButton(2)) {
//                m_servoLClutch.set(1);
//                m_servoRClutch.set(1);
//            }
//            else if (m_xBox1.getRawButton(3)) {
//                m_servoLClutch.set(0);
//                m_servoRClutch.set(0);
//            }
            
            
            if (m_xBox1.getRawButton(5)) { //I want the input from the D pad for all of these .getRawAxis
                // If Dpad is up, drives the motors one direction
                //m_spikeRelay2.set(Relay.Value.kOn);
                m_spikeBatteryMotor.set(Relay.Value.kForward);
            }
            else if (m_xBox1.getRawButton(6)) {
                // If Dpad is down, drives the motors the other direction
                //m_spikeRelay2.set(Relay.Value.kOn);
                m_spikeBatteryMotor.set(Relay.Value.kReverse);
            }
            else {
                //Turns off motors
                m_spikeBatteryMotor.set(Relay.Value.kOff);
            }
            
            
            if (m_xBox1.getRawButton(5) || m_xBox1.getRawButton(6)) {
                System.out.println("Left Bump " + m_xBox1.getRawButton(5) + " Right Bump " + m_xBox1.getRawButton(6));
            }
            
            //System.out.println("ServoLC position " + m_servoLClutch.getPosition() + " ServoRC position " + m_servoRClutch.getPosition());
            // System.out.println("Shooter Front " + m_shooterFrontSpeed + " Shooter Back " + m_shooterBackSpeed);
            
            
            m_shooterDriveFront.set(m_shooterFrontSpeed); //Shooter motors, front
            m_shooterDriveBack.set(m_shooterBackSpeed); //Shooter motors, back
            
            
            // LED turns on when within range
//            if (m_distanceCenterTarget < 200) {
//                m_spikeGreenLED.set(Relay.Value.kOn);
//            }
//            else {
//                m_spikeGreenLED.set(Relay.Value.kOff);
//            }
//            //Blinking light logic.
//            if (** Needs criteria here **) {
//                m_blink = true;
//            }
//            else {
//                m_blink = false;
//                m_spikeRedLED.set(Relay.Value.kOff);
//                m_blinkCounter = 0;
//            }
//            if (m_blink == true && m_lightOn == true) {
//                m_spikeRedLED.set(Relay.Value.kOn);
//                m_blinkCounter++;
//                if (m_blinkCounter >= 10) {
//                    m_lightOn = false;
//                    m_blinkCounter = 0;
//                }
//            }
//            else if (m_blink == true && m_lightOn == false) {
//                m_spikeRedLED.set(Relay.Value.kOff);
//                m_blinkCounter++;
//                if (m_blinkCounter >= 10) {
//                    m_lightOn = true;
//                    m_blinkCounter = 0;
//                }
//            }
            // Notes to self so I dont forget my ideas
            // If the robot sees more than one target, it will prioritize the high target over the middle target.
            // When the robot sees a target it will rotate towards the target, deaccelerating the rotation untill it stops on target,
            //  by either using a matrix or a series of if statements that will break up the returned value into sections, each section having a different
            //  corisponding level of power of rotation. 
            // EveryTime the robot sees a target, though the vision processing will update a global variable for each target which is then passed to this logic.
            // Targeting Logic
//            if (m_rotationCenterHigh > 0){
//                // all will rotate counter clock
//                if (m_rotationCenterHigh >= .75 && m_rotationCenterHigh < .5) {
//                    //set rotation 50%
//                }
//                else if (m_rotationCenterHigh >= .5 && m_rotationCenterHigh < .25) {
//                    //set rotation 40%
//                }
//                else if (m_rotationCenterHigh >= .25 && m_rotationCenterHigh < .1) {
//                    //set rotation 30%
//                }
//                else if (m_rotationCenterHigh >= .1) {
//                    //set rotation 15%
//                }
//            }
//            if (m_rotationCenterHigh < 0) {
//                // all will rotate clockwise
//                if (m_rotationCenterHigh <= -.75 && m_rotationCenterHigh > -.5) {
//                    //set rotation 50%
//                }
//                else if (m_rotationCenterHigh <= -.5 && m_rotationCenterHigh > -.25) {
//                    //set rotation 40%
//                }
//                else if (m_rotationCenterHigh <= -.25 && m_rotationCenterHigh > -.1) {
//                    //set rotation 30%
//                }
//                else if (m_rotationCenterHigh <= -.1) {
//                    //set rotation 15%
//                }
//            }
            
            // End Targeting Logic
            if (!m_xBox2.getRawButton(4) || !m_xBox2.getRawButton(3) || !m_xBox2.getRawButton(2)) {
                m_mecanumDrive.mecanumDrive_Polar(m_magnitude, m_direction, m_rotation);
            }
            
            
            //slow the loop.  
            //It must be at least (0.1) or better, sugguest a little faster inorder to not kill the robot for being too slow.
            Timer.delay(m_operatorControlLoopDelay);
            
            if (loopCount % 100 == 0 || loopCount == 1) {
                System.out.println("operatorControl Loop " + loopCount);
            }
            loopCount++;
            
        }
        
        stopRobot();  //Kill the motors on the robot
    
    }
    
    //FRC Default test()
    public void test() {
        clearScreen();
        updateScreen(1, "test()");
    }
    
    //overide the default default()
    public void disabled() {
        
        stopRobot();
        if (m_disabledCount > 0) {    
            clearScreen();
            updateScreen(1, "disabled()");
        }
        m_disabledCount++;
    }
    
    //scaled motor input
    public double scaledInput(double joyValue) {
        final double MAX_JOY_VAL = 1;    //the is the max value the joystick will output  
        final double MAX_MOTOR_VAL = 1;    //this is the max the motor input will take

        //calculate for scaled value  
        double direction = joyValue / Math.abs(joyValue);     // 1 or -1 to determin forwards or backwards  
        double ratio = ((joyValue * joyValue) / (MAX_JOY_VAL * MAX_JOY_VAL));    //log math  
        double scaledVal = (ratio * MAX_MOTOR_VAL) * direction;    //scaled output math

        return scaledVal;
    }
     
    //used to stop all moving robot parts and lights.
    public void stopRobot() {
        m_mecanumDrive.stopMotor();
        m_shooterDriveFront.set(0); //Konnor //Drive wheels for shooter 
        m_shooterDriveBack.set(0);
        m_spikeShooterAngle.set(Relay.Value.kOff);
        m_spikeBatteryMotor.set(Relay.Value.kOff);
        m_spikeFrisPusherMotor.set(Relay.Value.kOff);
      
    }
    
    //clear the screen
    public void clearScreen() {
        for (int i = 1; i <= 6; i++) {
            updateScreen(i, m_blankScreenStr);
        }
    }
    
    //function to update variables to be displayed
    public void updateScreen(int lineCount, String lineContent) {
//       lineContent = lineContent.substring(0, m_blankScreenStr.length()); //trim the output to the max length
       
        switch (lineCount) {
            case 1: m_screenLine1 = lineContent;
                    break;
            case 2: m_screenLine2 = lineContent;
                    break;
            case 3: m_screenLine3 = lineContent;
                    break;
            case 4: m_screenLine4 = lineContent;
                    break;
            case 5: m_screenLine5 = lineContent;
                    break;
            case 6:  m_screenLine6 = lineContent;
                    break;
            default: break;
        }
    }
    
    //update the screen Thread
    Runnable updateScreenTask = new Runnable()
    {
	public void run()
	{
            int loopCount = 1;
            
            while (true) {
                
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser1, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser1, 1, m_screenLine1);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser2, 1, m_screenLine2);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser3, 1, m_screenLine3);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser4, 1, m_screenLine4);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser5, 1, m_screenLine5);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, m_blankScreenStr);
                DriverStationLCD.getInstance().println(DriverStationLCD.Line.kUser6, 1, m_screenLine6);
                DriverStationLCD.getInstance().updateLCD();
                
                Timer.delay(m_screenLoopDelay);
                
                if (loopCount % 100 == 0 || loopCount == 1) {
                    System.out.println("Screen Loop " + loopCount);
                }
                loopCount++;
                
            }
        }
    };
    
    // Vision Processing thread
    Runnable visionProcessingTask = new Runnable()
    {
	public void run()
        {
            Timer.delay(2);
            //System.out.println("First line in RUN of VisionThread");
            int loopCount = 1;
            
            
            
	    while (true) {
                
                try {
                    // System.out.println("First line in TRY of VisionThread");
                   /**
                    * Do the image capture with the camera and apply the algorithm described above. This
                    * sample will either get images from the camera or from an image file stored in the top
                    * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
                    * 
                    */
                   ColorImage image = m_camera.getImage();     // comment if using stored images
                   //ColorImage image;                           // next 2 lines read image from flash on cRIO
                   //image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
                   BinaryImage thresholdImage = image.thresholdHSV(60, 100, 90, 255, 20, 255);   // keep only red objects
                   //thresholdImage.write("/threshold.bmp");
                   BinaryImage convexHullImage = thresholdImage.convexHull(false);          // fill in occluded rectangles
                   //convexHullImage.write("/convexHull.bmp");
                   BinaryImage filteredImage = convexHullImage.particleFilter(m_cc);           // filter out small particles
                   //filteredImage.write("/filteredImage.bmp");

                   //System.out.println("After variable declaration TRY of VisionThread");
                   //iterate through each particle and score to see if it is a target
                  FRC2013Team2037_Main.Scores scores[] = new FRC2013Team2037_Main.Scores[filteredImage.getNumberParticles()];
                   for (int i = 0; i < scores.length; i++) {
                       //System.out.println("First line in FOR loop of VisionThread");
                       ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                       scores[i] = new Scores();

                       scores[i].rectangularity = scoreRectangularity(report);
                       scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,report, i, true);
                       scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, i, false);
                       scores[i].xEdge = scoreXEdge(thresholdImage, report);
                       scores[i].yEdge = scoreYEdge(thresholdImage, report);

                       if(scoreCompare(scores[i], false))
                       {
                           //m_highTargetVisible = true;
                           m_rotationCenterHigh = report.center_mass_x_normalized;
                           System.out.println("particle: " + i + " is a High Goal  centerX: " + m_rotationCenterHigh + " centerY: " + report.center_mass_y_normalized);
                           
                           double m_localDistance = computeDistance(thresholdImage, report, i, false);
                          if (m_localDistance < (m_distanceCenterTarget * 1.25)){
                           m_average = (m_localDistance + m_tempVariable + m_distanceCenterTarget)/3;
                           m_distanceCenterTarget = m_tempVariable;
                           m_tempVariable = m_localDistance;
                           System.out.println("Distance: " + m_distanceCenterTarget); 
                          }
                          updateScreen(2,("TargetDis: " + m_distanceCenterTarget) );
                           System.out.println("rect: " + scores[i].rectangularity + " ARinner: " + scores[i].aspectRatioInner);
                           System.out.println("ARouter: " + scores[i].aspectRatioOuter + " xEdge: " + scores[i].xEdge + " yEdge: " + scores[i].yEdge);
                         

                       } else if (scoreCompare(scores[i], true)) {
                           //m_middleTargetVisible = true;
                           m_rotationCenterMiddle = report.center_mass_x_normalized; 
                           System.out.println("particle: " + i + " is a Middle Goal  centerX: " + m_rotationCenterMiddle + " centerY: " + report.center_mass_y_normalized);
                           System.out.println("Distance: " + computeDistance(thresholdImage, report, i, true));
                           System.out.println("rect: " + scores[i].rectangularity + " ARinner: " + scores[i].aspectRatioInner);
                           System.out.println("ARouter: " + scores[i].aspectRatioOuter + " xEdge: " + scores[i].xEdge + " yEdge: " + scores[i].yEdge);
                          
                            

                       } else {
                           System.out.println("particle: " + i + " is not a goal");
   //                        System.out.println("particle: " + i + "is not a goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
   //                        System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
   //			  System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);
                            
                       }
                       

                   }
                   //System.out.println("Outside FOR loop of VisionThread");
                   /**
                    * all images in Java must be freed after they are used since they are allocated out
                    * of C data structures. Not calling free() will cause the memory to accumulate over
                    * each pass of this loop.
                    */
                   filteredImage.free();
                   convexHullImage.free();
                   thresholdImage.free();
                   image.free();

               } 
                catch (AxisCameraException ex) {        // this is needed if the camera.getImage() is called
                   ex.printStackTrace();
               } 
                catch (NIVisionException ex) {
                   ex.printStackTrace();
               }
                   
                Timer.delay(m_visionLoopDelay);
                if (loopCount % 100 == 0 || loopCount == 1) {
                    System.out.println("Vision Loop " + loopCount);
                }
                loopCount++;
                
          }
                 
        }

    };
    

    double computeDistance (BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean outer) throws NIVisionException {
            double rectShort, height;
            int targetHeight;

            rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
            //using the smaller of the estimated rectangle short side and the bounding rectangle height results in better performance
            //on skewed rectangles
            height = Math.min(report.boundingRectHeight, rectShort);
            targetHeight = outer ? 29 : 21;

            return X_IMAGE_RES * targetHeight / (height * 12 * 2 * Math.tan(VIEW_ANGLE*Math.PI/(180*2)));
    }
    
    // Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
    // the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
    // to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
    // and particle perimeter= 2x+2y
    // 
    // @param image The image containing the particle to score, needed to performa additional measurements
    // @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
    // @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
    // @return The aspect ratio score (0-100)
     
    public double scoreAspectRatio(BinaryImage image, ParticleAnalysisReport report, int particleNumber, boolean outer) throws NIVisionException
    {
        double rectLong, rectShort, aspectRatio, idealAspectRatio;

        rectLong = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_LONG_SIDE);
        rectShort = NIVision.MeasureParticle(image.image, particleNumber, false, NIVision.MeasurementType.IMAQ_MT_EQUIVALENT_RECT_SHORT_SIDE);
        idealAspectRatio = outer ? (62/29) : (62/20);	//Dimensions of goal opening + 4 inches on all 4 sides for reflective tape
	
        //Divide width by height to measure aspect ratio
        if(report.boundingRectWidth > report.boundingRectHeight){
            //particle is wider than it is tall, divide long by short
            aspectRatio = 100*(1-Math.abs((1-((rectLong/rectShort)/idealAspectRatio))));
        } else {
            //particle is taller than it is wide, divide short by long
                aspectRatio = 100*(1-Math.abs((1-((rectShort/rectLong)/idealAspectRatio))));
        }
	return (Math.max(0, Math.min(aspectRatio, 100.0)));		//force to be in range 0-100
    }
    
    // Compares scores to defined limits and returns true if the particle appears to be a target
    // 
    //@param scores The structure containing the scores to compare
    //@param outer True if the particle should be treated as an outer target, false to treat it as a center target
    // 
    //@return True if the particle meets all limits, false otherwise
    boolean scoreCompare(FRC2013Team2037_Main.Scores scores, boolean outer){
            boolean isTarget = true;

            isTarget &= scores.rectangularity > RECTANGULARITY_LIMIT;
            if(outer){
                    isTarget &= scores.aspectRatioOuter > ASPECT_RATIO_LIMIT;
            } else {
                    isTarget &= scores.aspectRatioInner > ASPECT_RATIO_LIMIT;
            }
            isTarget &= scores.xEdge > X_EDGE_LIMIT;
            isTarget &= scores.yEdge > Y_EDGE_LIMIT;

            return isTarget;
    }
    
    //Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
    //to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
    // 
    //@param report The Particle Analysis Report for the particle to score
    //@return The rectangularity score (0-100)
    double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }	
    }
    
    //Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
    ///the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
    //a hollow center.
    //
    //@param image The image to use, should be the image before the convex hull is performed
    //@param report The Particle Analysis Report for the particle
    // 
    //@return The X Edge Score (0-100)
    public double scoreXEdge(BinaryImage image, ParticleAnalysisReport report) throws NIVisionException
    {
        double total = 0;
        LinearAverages averages;
        
        NIVision.Rect rect = new NIVision.Rect(report.boundingRectTop, report.boundingRectLeft, report.boundingRectHeight, report.boundingRectWidth);
        averages = NIVision.getLinearAverages(image.image, LinearAverages.LinearAveragesMode.IMAQ_COLUMN_AVERAGES, rect);
        float columnAverages[] = averages.getColumnAverages();
        for(int i=0; i < (columnAverages.length); i++){
                if(xMin[(i*(XMINSIZE-1)/columnAverages.length)] < columnAverages[i] 
                   && columnAverages[i] < xMax[i*(XMAXSIZE-1)/columnAverages.length]){
                        total++;
                }
        }
        total = 100*total/(columnAverages.length);
        return total;
    }
    
    //Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
    //the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
    //a hollow center
    //
    //@param image The image to use, should be the image before the convex hull is performed
    //@param report The Particle Analysis Report for the particle
    //
    //@return The Y Edge score (0-100)
    public double scoreYEdge(BinaryImage image, ParticleAnalysisReport report) throws NIVisionException
    {
        double total = 0;
        LinearAverages averages;
        
        NIVision.Rect rect = new NIVision.Rect(report.boundingRectTop, report.boundingRectLeft, report.boundingRectHeight, report.boundingRectWidth);
        averages = NIVision.getLinearAverages(image.image, LinearAverages.LinearAveragesMode.IMAQ_ROW_AVERAGES, rect);
        float rowAverages[] = averages.getRowAverages();
        for(int i=0; i < (rowAverages.length); i++){
                if(yMin[(i*(YMINSIZE-1)/rowAverages.length)] < rowAverages[i] 
                   && rowAverages[i] < yMax[i*(YMAXSIZE-1)/rowAverages.length]){
                        total++;
                }
        }
        total = 100*total/(rowAverages.length);
        return total;
    }
    
    
}