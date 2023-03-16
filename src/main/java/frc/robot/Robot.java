// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.io.IOException;
import java.nio.file.Path;

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

  String trajectoryJSON = "Paths/Test path to get to save zone.wpilib.json";
  Trajectory trajectory = new Trajectory();

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
    defaultLevel = ahrs.getRoll();
    currentLevel = ahrs.getRoll();

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
}
