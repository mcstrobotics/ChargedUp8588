// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * Names: Ethan and Nishit 
 * Started March 7
 * A lot of the variables may be deleted soon depending on what approach we take :/ 
 * Uhhh... idk what else to put in this heder file 
 */
package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private final double autonPeriod = 15;
  private AHRS ahrs;
  private Timer timer;
  private float defaultLevel = ahrs.getRoll();;
  private float currentLevel;

  private DriveSubsystem driveSubsystem;
  private DriveDirection driveDirection;
  private DriveCommand driveCommand;
  private TankDriveInputs tankInputs;
  private TankDriveChassis tankhassis;
  private TankDriveSubsystem tankSubsystem;



  // Define the reference orientation and the tilt angle threshold
  private double refAngle = 0;
  private double tiltThreshold = 5; // in degrees

  // Define the forward and backward speed limits
  private double forwardSpeedLimit = 0.5;
  private double backwardSpeedLimit = -0.5;

  // Define the PID controller gains
  private double kP = 0.1;
  private double kI = 0.0;
  private double kD = 0.0;
  private double integral = 0;
  private double previousError = 0;

  private double speed;
  private double pidOutput; 
  

  private enum AutonomousPhase {
    PHASE1_DROP_PAYLOAD,
    PHASE2_MOVE_OUT_OF_SAFE_ZONE,
    PHASE3_LOCATE_DOCK,
    PHASE4_MOVE_TOWARD_DOCK,
    PHASE5_GET_ON_DOCK,
    PHASE6_LEVEL
  }

  private AutonomousPhase currentPhase;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    timer = new Timer();
    currentPhase = AutonomousPhase.PHASE1_DROP_PAYLOAD;


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    currentLevel = ahrs.getRoll();

    // Calculate the tilt angle with respect to the reference orientation
    double tiltAngle = currentLevel - defaultLevel;
    // Adjust the robot's movements based on the tilt angle
    double forwardSpeed = forwardSpeedLimit;
    double backwardSpeed = backwardSpeedLimit;
    if (tiltAngle > tiltThreshold) {
      forwardSpeed = 0;
    } 
    else if (tiltAngle < -tiltThreshold) {
      backwardSpeed = 0;
    }

    // Fine-tune the robot's movements using a PID controller
    double error = tiltAngle;
    integral += error * 0.02; // integrate over 20ms (default loop time)
    double derivative = (error - previousError) / 0.02; // differentiate over 20ms
    pidOutput = kP * error + kI * integral + kD * derivative;
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    timer.reset();
    timer.start();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double timeElapsed = timer.get();


    if (timeElapsed < autonPeriod) {
            switch (currentPhase) {
                case PHASE1_DROP_PAYLOAD:
                    // do any necessary pre-phase setup
                    currentPhase = AutonomousPhase.PHASE2_MOVE_OUT_OF_SAFE_ZONE;
                    break;
                    
                case PHASE2_MOVE_OUT_OF_SAFE_ZONE:
                    // locate the dock pad
                    // move toward the dock pad
                    // drop the payload into the dock
                    currentPhase = AutonomousPhase.PHASE3_LOCATE_DOCK;
                    break;
                    
                case PHASE3_LOCATE_DOCK:
                    // re-orientate if necessary
                    // move out of the safe zone
                    // periodically check for obstacles
                    // if an obstacle is identified, go around it and continue on the path
                    currentPhase = AutonomousPhase.PHASE4_MOVE_TOWARD_DOCK;
                    break;
                    
                case PHASE4_MOVE_TOWARD_DOCK:
                    // locate the dock using the camera
                    // identify corners and sides
                    // generate a path to the dock
                    currentPhase = AutonomousPhase.PHASE5_GET_ON_DOCK;
                    break;
                    
                case PHASE5_GET_ON_DOCK:
                    // move along the path to the dock
                    // periodically check for obstacles
                    // if an obstacle is identified, go around it and continue on the path
                    currentPhase = AutonomousPhase.PHASE6_LEVEL;
                    break;
                
                case PHASE6_LEVEL:
                if (currentLevel < defaultLevel) {
                  speed *= (1 - pidOutput);
                  tankSubsystem.drive(speed, DriveDirection.FORWARD);
                  if (currentLevel > defaultLevel) {
                    speed *= (1 + pidOutput);
                    tankSubsystem.drive(speed, DriveDirection.BACKWARD);
                  }
                    
      }
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /*this method detects colors of either the cones, cubes,  and bumpers of other robots (red or blue) and puts their
  contours as rectangles to be drawn on the main image*/
  public void detectContours(Mat img, Scalar lower, Scalar higher, double aspRatio, ArrayList<Rect> rectangles){
    ArrayList<MatOfPoint> contours = new ArrayList<MatOfPoint>(); //find contours
    Mat hierarchy = new Mat(); //hierarchy, for the color isolation
    Mat dest = new Mat(); //destination of the color alterred image 
    Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5)); /*kernel for dialation, 
    erosion,opening, etc. Am i creating too many mats!?!*/

    //color isolation
    Imgproc.cvtColor(img, dest, Imgproc.COLOR_BGR2HSV); //transform image into an HSV image 
    Core.inRange(dest, lower, higher, dest); //get color

    // remove noise via opening 
    Imgproc.morphologyEx(dest, dest, Imgproc.MORPH_OPEN, kernel);
    //isolate contours
    Imgproc.findContours(dest, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
    MatOfPoint2f[] contoursReg = new MatOfPoint2f[contours.size()]; //approx contour length,(good for boundng box)
    /*further get rid of noise/regularify the sides, 
    plus we need to convert our countors to a different format so we can create bounding rectangles! */
    
    for(int i = 0; i < contours.size(); i++){
      double area = Imgproc.contourArea(contours.get(i));

      if(area > 100){ //if the contour isn't too small (test for noise later on)
        Imgproc.drawContours(img, contours, -1, this.boxColor, 4); //draw contours(for dashboard) 

        double perimeter = Imgproc.arcLength(new MatOfPoint2f(contours.get(i).toArray()), true); //get the perimeter
        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursReg[i], 0.02 * perimeter, true); //polygonify
        Rect potential = Imgproc.boundingRect(new MatOfPoint(contoursReg[i].toArray()));  //use to check aspect ratio


        //check if the desired aspect ratio is met
        double ratio = ((double)potential.width) / ((double)potential.height); //calculate the aspect ratio of the rectangle
        if(ratio > (aspRatio * 0.95) && ratio < (aspRatio * 1.05)){ //if the aspect ratio is close enough
          rectangles.add(potential); //add it to the list! 
        }
      }
    }
  }

  public void findObjects(){ //for simplicity we have a method that scans for EVERYTHING we are looking for 
    this.detectContours(feed, coneLower, coneHigher, coneAspectRatio, payloadBoundingRect); //scan for cubes
    this.detectContours(feed, cubeLower, cubeHigher, cubeAspectRatio, payloadBoundingRect); //scan for cones
    this.detectContours(feed, redLower, redHigher, robotAspectRatio, robotBoundingRect); //scan for red team robots
    this.detectContours(feed, blueLower, blueHigher, robotAspectRatio, robotBoundingRect); //scan for blue team robots

  }
  /*periodically scan for obstacles to see if there are within the robot's range to be hit 
  if the obstacle is a bumper(stand in place ig?) if it is a cone/cube move around it ig*/
  public void scanObstacles(Mat img){

  }
}
