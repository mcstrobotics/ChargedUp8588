// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

//Autonomonous Import
import edu.wpi.first.cameraserver.CameraServer; //camera server
import edu.wpi.first.cscore.UsbCamera; //usb camera
import edu.wpi.first.cscore.CvSink; //sink 
import edu.wpi.first.cscore.CvSource; //source, yadda yadda, all of this vision processing stuff :P 
import edu.wpi.first.cscore.MjpegServer; //i have no idea if we want to creat mjpeg servers tbh, or if this is even needed
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
  private Timer timer;

  private enum AutonomousPhase {
    PHASE1_DROP_PAYLOAD,
    PHASE2_MOVE_OUT_OF_SAFE_ZONE,
    PHASE3_LOCATE_DOCK,
    PHASE4_MOVE_TOWARD_DOCK,
    PHASE5_GET_ON_DOCK,
    PHASE6_LEVEL
  }

  private AutonomousPhase currentPhase;

  //Variables for image processing (capitalized variabels arent finalized yet until we can collect data)
  private static int IMG_WIDTH; //width of the image being captured (not finalizing until we get a value for our resolution)
  private static int IMG_HEIGHT; //height/length of captured image 

  private static double FIELD_OF_VIEW; //feild of view of our camera(calculate manually)
  private static int CAMERA_ANGLE; //angle the camera is mounted on
  private static int CAMERA_HEIGHT; //how high up is our camera mounted? 

  //help us with the positioning of our target 
  private static double pitch; //pitch, important for distance judging
  private static double yaw; //tbh idk where we would use the yaw :/ 
  private static int targetPosX; //use this to write poition in coordingates to coordinate grid to match driving/joystick coordinate plane
  private static int targetPosY; // likewise as aboove 

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

    //initialize our camera(should be connected to Robot RIO, will double check that we are using one with engineering/electrical)
    UsbCamera camera = CameraServer.startAutomaticCapture();
    //set the resolution of the camera
    camera.setResolution(IMG_WIDTH, IMG_HEIGHT); //set the resolution

    //Insert CV source and sink (later, I would like to do some more research to figure out what those do)
    CvSink sink = CameraServer.getVideo(); //get images for processing(sink)
    CvSource output = CameraServer.putVideo("DevilCam", IMG_WIDTH, IMG_HEIGHT); //in case we want to send anything to the dashboard
    
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
}
