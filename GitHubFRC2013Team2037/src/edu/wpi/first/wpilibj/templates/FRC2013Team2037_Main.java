/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

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
     * 1. m_spikeRelay
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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SimpleRobot;
import edu.wpi.first.wpilibj.Timer;
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
    
    Joystick m_xBox1 = new Joystick(1);  //driver joystick, controller 1
    Joystick m_xBox2 = new Joystick(2);  //shooter joystick, controller 2
    //robot Drive init, motor input order is frontLeft, rearLeft, frontRight, rearRight
    //robot Drive actual layout is frontLeft, frontRight, rearRight, rearLeft
    RobotDrive m_mecanumDrive = new RobotDrive(1,4,2,3); 
    DigitalInput m_microSwitch = new DigitalInput(1);  //microSwitch1
    Gyro m_gyro = new Gyro(1);
    Relay m_spikeRelay = new Relay(1);  //spikeRelay to blink a light via microSwitch1
    
    
    //Global Variables
    double m_magnitude;
    double m_direction;
    double m_rotation;             
    //double m_change;      for temp    
    //double m_temperature; for temp
    double m_gyroDataStart;
    double m_gyroDataCurrent;
    double m_gyroSensitivity = 0.20;
    
    
    
    //vision code.
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
    
    AxisCamera camera;          // the axis camera object (connected to the switch)
    CriteriaCollection cc;      // the criteria for doing the particle filter operation
    
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
        m_mecanumDrive.setInvertedMotor(RobotDrive.MotorType.kFrontRight, true);
        m_mecanumDrive.setInvertedMotor(RobotDrive.MotorType.kRearRight, true);
        m_spikeRelay.set(Relay.Value.kOff);
        m_gyro.reset();
        Timer.delay(.5);
    }

    
    
    //This function is called once each time the robot enters autonomous mode.
    public void autonomous() {
            
        while (isAutonomous() && isEnabled()) {
           
            m_mecanumDrive.drive(.5, .5);
           
        }
        
        stopRobot();
    }

    //This function is called once each time the robot enters operator control.
    public void operatorControl() {
        
        double m_xb1DeadZone = 0.165;  //we need to play with this number to see what needs to be changed
        double m_xb1_ax1;
        double m_xb1_ax2;
        double m_xb1_ax3;
        double m_xb1_ax4;
        double m_xb1_ax5;
        double m_xb1_ax6;
        double m_slowMotorSpeed;

        
        
        while (isOperatorControl() && isEnabled()) {
            
            
            m_slowMotorSpeed = .5;
            m_gyro.setSensitivity(m_gyroSensitivity);  //use to slow the number down. 360 rotation equals (fill in with correct number)
        
            
            
            //m_mecanumDrive.setSafetyEnabled(false);
            
                        
            //reads the current gyro data
            m_gyroDataCurrent = m_gyro.getAngle();
            
            if(m_xBox1.getBumper(GenericHID.Hand.kRight) == true) {
                m_slowMotorSpeed = 1;
            }
               
            //this is from the left joystick
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
            if (Math.abs(m_xBox1.getDirectionDegrees()) > 0) //m_deadZone)
            {
                m_direction = m_xBox1.getDirectionDegrees();
            }
            else
            {
                m_direction = 0;
            }

            //this is from the right joystick
            if (Math.abs(m_xBox1.getX(GenericHID.Hand.kRight)) > m_xb1DeadZone)
            {
                m_rotation = scaledInput(m_xBox1.getRawAxis(4));
                //m_rotation = m_rotation * m_slowDownRate;  //add if we need to slow her down.
            }
            else
            {
                m_rotation = 0;
            }
           
            
            
            //level all joystick inputs, run through scaling code
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
//            //Uncomment to see more debug lines
//            System.out.println("MicroSwitch says.... " + m_microSwitch.get());
//            System.out.println("SpikeRelay says....  " + m_spikeRelay.get());
//            System.out.println("Gyro Says....        " + m_gyroDataCurrent);
            
            
            //microSwitch and spikeRelay testing code
            if (m_microSwitch.get() == false){
                m_spikeRelay.set(Relay.Value.kOff);
            }
            else {
                m_spikeRelay.setDirection(Relay.Direction.kForward);
                m_spikeRelay.set(Relay.Value.kOn);
            }
            
            
            m_mecanumDrive.mecanumDrive_Polar(m_magnitude, m_direction, m_rotation);
            
            
            //slow the loop for display reasons.
            Timer.delay(.02);
        }
        
        stopRobot();
    
    }
    
    //FRC Default test()
    public void test() {
    
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
        m_spikeRelay.set(Relay.Value.kOff);
      
    }
    
  

    
    public void mainVisionProcessing() {
        
    
        try {
                /**
                 * Do the image capture with the camera and apply the algorithm described above. This
                 * sample will either get images from the camera or from an image file stored in the top
                 * level directory in the flash memory on the cRIO. The file name in this case is "testImage.jpg"
                 * 
                 */
                ColorImage image = camera.getImage();     // comment if using stored images
                //ColorImage image;                           // next 2 lines read image from flash on cRIO
                //image = new RGBImage("/testImage.jpg");		// get the sample image from the cRIO flash
                BinaryImage thresholdImage = image.thresholdHSV(60, 100, 90, 255, 20, 255);   // keep only red objects
                //thresholdImage.write("/threshold.bmp");
                BinaryImage convexHullImage = thresholdImage.convexHull(false);          // fill in occluded rectangles
                //convexHullImage.write("/convexHull.bmp");
                BinaryImage filteredImage = convexHullImage.particleFilter(cc);           // filter out small particles
                //filteredImage.write("/filteredImage.bmp");
                
                
                //iterate through each particle and score to see if it is a target
               FRC2013Team2037_Main.Scores scores[] = new FRC2013Team2037_Main.Scores[filteredImage.getNumberParticles()];
                for (int i = 0; i < scores.length; i++) {
                    ParticleAnalysisReport report = filteredImage.getParticleAnalysisReport(i);
                    scores[i] = new Scores();
                    
                    scores[i].rectangularity = scoreRectangularity(report);
                    scores[i].aspectRatioOuter = scoreAspectRatio(filteredImage,report, i, true);
                    scores[i].aspectRatioInner = scoreAspectRatio(filteredImage, report, i, false);
                    scores[i].xEdge = scoreXEdge(thresholdImage, report);
                    scores[i].yEdge = scoreYEdge(thresholdImage, report);

                    if(scoreCompare(scores[i], false))
                    {
                          
                        System.out.println("particle: " + i + "is a High Goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
			System.out.println("Distance: " + computeDistance(thresholdImage, report, i, false));
                        System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
			System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);
                        
                    } else if (scoreCompare(scores[i], true)) {
                        
                        
			System.out.println("particle: " + i + "is a Middle Goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
			System.out.println("Distance: " + computeDistance(thresholdImage, report, i, true));
                        System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
			System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);
                        
                    } else {
                        
                        
//                        System.out.println("particle: " + i + "is not a goal  centerX: " + report.center_mass_x_normalized + "centerY: " + report.center_mass_y_normalized);
//                        System.out.println("rect: " + scores[i].rectangularity + "ARinner: " + scores[i].aspectRatioInner);
//			  System.out.println("ARouter: " + scores[i].aspectRatioOuter + "xEdge: " + scores[i].xEdge + "yEdge: " + scores[i].yEdge);
                        
                    }
			
                }

                /**
                 * all images in Java must be freed after they are used since they are allocated out
                 * of C data structures. Not calling free() will cause the memory to accumulate over
                 * each pass of this loop.
                 */
                filteredImage.free();
                convexHullImage.free();
                thresholdImage.free();
                image.free();
                
            } catch (AxisCameraException ex) {        // this is needed if the camera.getImage() is called
                ex.printStackTrace();
            } catch (NIVisionException ex) {
                ex.printStackTrace();
            }
    }
    
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
    
    /**
     * Computes a score (0-100) comparing the aspect ratio to the ideal aspect ratio for the target. This method uses
     * the equivalent rectangle sides to determine aspect ratio as it performs better as the target gets skewed by moving
     * to the left or right. The equivalent rectangle is the rectangle with sides x and y where particle area= x*y
     * and particle perimeter= 2x+2y
     * 
     * @param image The image containing the particle to score, needed to performa additional measurements
     * @param report The Particle Analysis Report for the particle, used for the width, height, and particle number
     * @param outer	Indicates whether the particle aspect ratio should be compared to the ratio for the inner target or the outer
     * @return The aspect ratio score (0-100)
     */
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
    
    /**
     * Compares scores to defined limits and returns true if the particle appears to be a target
     * 
     * @param scores The structure containing the scores to compare
     * @param outer True if the particle should be treated as an outer target, false to treat it as a center target
     * 
     * @return True if the particle meets all limits, false otherwise
     */
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
    
    /**
     * Computes a score (0-100) estimating how rectangular the particle is by comparing the area of the particle
     * to the area of the bounding box surrounding it. A perfect rectangle would cover the entire bounding box.
     * 
     * @param report The Particle Analysis Report for the particle to score
     * @return The rectangularity score (0-100)
     */
    double scoreRectangularity(ParticleAnalysisReport report){
            if(report.boundingRectWidth*report.boundingRectHeight !=0){
                    return 100*report.particleArea/(report.boundingRectWidth*report.boundingRectHeight);
            } else {
                    return 0;
            }	
    }
    
    /**
     * Computes a score based on the match between a template profile and the particle profile in the X direction. This method uses the
     * the column averages and the profile defined at the top of the sample to look for the solid vertical edges with
     * a hollow center.
     * 
     * @param image The image to use, should be the image before the convex hull is performed
     * @param report The Particle Analysis Report for the particle
     * 
     * @return The X Edge Score (0-100)
     */
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
    
    /**
	 * Computes a score based on the match between a template profile and the particle profile in the Y direction. This method uses the
	 * the row averages and the profile defined at the top of the sample to look for the solid horizontal edges with
	 * a hollow center
	 * 
	 * @param image The image to use, should be the image before the convex hull is performed
	 * @param report The Particle Analysis Report for the particle
	 * 
	 * @return The Y Edge score (0-100)
	 *
    */
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
