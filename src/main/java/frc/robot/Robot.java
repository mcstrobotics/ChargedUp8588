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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.usercontrol.HOTASJoystick;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.subsystems.drive.DriveSubsystem; 
import frc.robot.subsystems.drive.DriveDirection; 
import frc.robot.subsystems.drive.tank.TankDriveInputs;
import frc.robot.subsystems.drive.tank.TankDriveChassis;
import frc.robot.subsystems.drive.tank.TankDriveSubsystem;
import frc.robot.commands.DriveCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera; 
import edu.wpi.first.cscore.CvSink;  
import edu.wpi.first.cscore.CvSource; 
import edu.wpi.first.cscore.MjpegServer; 
import org.opencv.core.Rect;
import org.opencv.core.Core;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc; //why cant i use one import for all opencv functionality!?! 


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

  String trajectoryJSON = "Paths/Test path to get to save zone.wpilib.json";
  Trajectory trajectory = new Trajectory();
  private static int IMG_WIDTH;
  private static int IMG_HEIGHT;

  //i uncommented these out cuz i thought we didnt need them but these may come back :/ 
  /* 
  private static double FIELD_OF_VIEW; //feild of view of our camera(calculate manually)
  private static int CAMERA_ANGLE; //angle the camera is mounted on
  private static int CAMERA_HEIGHT; //how high up is our camera mounted? 

  //help us with the positioning of our target 
  private static double pitch; //pitch, important for distance judging
  private static double yaw; //tbh idk where we would use the yaw :/ 
  private static int targetPosX; //use this to write poition in coordingates to coordinate grid to match driving/joystick coordinate plane
  private static int targetPosY; // likewise as aboove 
*/
  //Open CV/img processing stuff
  private Mat feed; //let's have this mat have the camera feed
  Scalar boxColor = new Scalar(0, 255, 0); //color of drawn contours 
  /*
   * pretty much to see if an obstacle is directly in front of the robot we will use a specific range on the camera feed to see
   * when an object is directly in the path/grabbable by intake so that way the robot can take action accordingly 
   */
  private static int frontCoordinate[] = {0,0}; //coordinate of range(top left)
  private static int frontDimensions[] = {0,0}; //(width and height of that box)

  //hue saturation and lightness range(Scalar(hue, saturation, value))
  //cone hue saturation lightness range (red)
  private Scalar coneLower = new Scalar(0,0,0);
  private Scalar coneHigher = new Scalar(0, 0, 0);
  //cube hue saturation lightness range(purple)
  private Scalar cubeLower = new Scalar(0,0,0);
  private Scalar cubeHigher = new Scalar(0, 0, 0);
  //red bumper and target lightness range (use aspect ratio to differentiate the objects)
  private Scalar redLower = new Scalar(0,0,0);
  private Scalar redHigher = new Scalar(0, 0, 0);
  //identify blue bumper and target lightness range(again use aspect ratio to differentiate)
  private Scalar blueLower = new Scalar(0,0,0);
  private Scalar blueHigher = new Scalar(0, 0, 0);

  //aspect ratio values (width/height)
  private double coneAspectRatio = 8 / 12.5; //aspect ratio of cone (width could potentially be 6 depending on what is picked up)
  private double cubeAspectRatio = 9.5 / 9.5; //aspect ratio of cube
  private double robotAspectRatio;
  private double lineAspectRatio;  //aspect ratio of lines(seperating home base)
  private double payloadAspectRatio; //aspect ratio of the payload docking station(however we choose to identify it)

  //countour arraylists
  ArrayList<Rect> payloadBoundingRect = new ArrayList<Rect>(10); //cones and cubes
  ArrayList<Rect> robotBoundingRect = new ArrayList<Rect>(10); //other robots 

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer(ahrs);
    timer = new Timer();
    currentPhase = AutonomousPhase.PHASE1_DROP_PAYLOAD;
    defaultLevel = ahrs.getRoll();
    currentLevel = ahrs.getRoll();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    //set the resolution of the camera
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT); //set the resolution

    try {
      ahrs = new AHRS(SPI.Port.kMXP); //the kMXP port is the expansion port for the roborio
      ahrs.enableLogging(true);
      //ahrs.calibrate(); //takes approximately 15 seconds to finish (leave commented out for now)
    }
    catch (RuntimeException ex) {
      DriverStation.reportError("Error creating navx sensor object! " + ex.getMessage(), true);
    }

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
   } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
   }
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
    m_autonomousCommand = m_robotContainer.getAutonCommand();
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
    int listCount = 0; //array list 

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
          if(listCount < rectangles.size()){ //scan for overflow
            rectangles.set(listCount, potential); //add it to the list! 
            listCount ++; //increase the count 
          }
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
  //periodically scan for obstacles to see if there are within the robot's range to be hit (stationary objects)
  public boolean scanStationaryObstacles(){
    for(int i = 0; i < payloadBoundingRect.size(); i++){ //run thru the arraylist of bounding rectangles 
      Rect check = payloadBoundingRect.get(i); //i want to type less >:P 
      if((check.x >= frontCoordinate[0] && check.y >= frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && check.y <= (frontCoordinate[1] + frontDimensions[1])) 
      ||(check.x >= frontCoordinate[0] && (check.y + check.height) >= frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) <= (frontCoordinate[1] + frontDimensions[1]))
      || ((check.x + check.width) > frontCoordinate[0] && (check.y + check.height) > frontCoordinate[1] && (check.x + check.width) <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) >= (frontCoordinate[1] + frontDimensions[1]))
      || check.x > frontCoordinate[0] && (check.y + check.height) > frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) <= (frontCoordinate[1] + frontDimensions[1]))
      { //long if statement to check the corners to see if there are in the robot's "frontal vicinity"
        return true; 
      }
    }
    return false; 
  }

  //same thing but for robots(we wnat to do different things if it is a robot or object)
  public boolean scanRobots(){
    for(int i = 0; i < payloadBoundingRect.size(); i++){ //run thru the arraylist of bounding rectangles 
      Rect check = payloadBoundingRect.get(i); //i want to type less >:P 
      if((check.x >= frontCoordinate[0] && check.y >= frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && check.y <= (frontCoordinate[1] + frontDimensions[1])) 
      ||(check.x >= frontCoordinate[0] && (check.y + check.height) >= frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) <= (frontCoordinate[1] + frontDimensions[1]))
      || ((check.x + check.width) > frontCoordinate[0] && (check.y + check.height) > frontCoordinate[1] && (check.x + check.width) <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) >= (frontCoordinate[1] + frontDimensions[1]))
      || check.x > frontCoordinate[0] && (check.y + check.height) > frontCoordinate[1] && check.x <= (frontCoordinate[0] + frontDimensions[0]) && (check.y + check.height) <= (frontCoordinate[1] + frontDimensions[1]))
      { //long if statement to check the corners to see if there are in the robot's "frontal vicinity"
        return true; 
      }
    }
    return false; 
  }
}
