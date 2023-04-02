// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutonCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.drive.DiffyDriveChassis;
import frc.robot.subsystems.drive.DiffyDriveInputs;
import frc.robot.subsystems.drive.DiffyDriveSubsystem;
import frc.robot.subsystems.intake.IntakeChassis;
import frc.robot.subsystems.intake.IntakeInputs;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.usercontrol.GamepadF310;
import frc.robot.usercontrol.HOTASJoystick;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // HOTAS Flight Stick
  //private HOTASJoystick flightStick = new HOTASJoystick(1);

  // good old F310
  private GamepadF310 f310 = new GamepadF310(1);

  private XboxController xboxController = new XboxController(0);

  public DiffyDriveSubsystem driveSubsystem = new DiffyDriveSubsystem(
      new DiffyDriveChassis(
          new CANSparkMax(Constants.kFrontRight, CANSparkMaxLowLevel.MotorType.kBrushless), // front right
          new CANSparkMax(Constants.kFrontLeft, CANSparkMaxLowLevel.MotorType.kBrushless), // front left
          new CANSparkMax(Constants.kBackRight, CANSparkMaxLowLevel.MotorType.kBrushless), // back right
          new CANSparkMax(Constants.kBackLeft, CANSparkMaxLowLevel.MotorType.kBrushless) // back left
      ),
      new DiffyDriveInputs(xboxController::getLeftY, xboxController::getRightX)); // x and y
                                                                                                            // of stick

  private IntakeSubsystem intakeSubsystem = new IntakeSubsystem(new IntakeChassis(
      new CANSparkMax(Constants.kArm, CANSparkMaxLowLevel.MotorType.kBrushless), // arm
      new CANSparkMax(Constants.kIntake, CANSparkMaxLowLevel.MotorType.kBrushless)),
      new IntakeInputs(f310::getLeftY, f310::getRightY)); // intake
  private DriveCommand driveCommand = new DriveCommand(driveSubsystem, intakeSubsystem); // issue the drive commands
                                                                                         // from the drive subsystem
  private IntakeCommand intakeCommand = new IntakeCommand(intakeSubsystem);

  Robot obj = new Robot();

  private AutonCommand autonCommand = new AutonCommand(driveSubsystem, obj.startTime);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // TODO Configure the trigger bindings
    // configureBindings(ahrs);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule commands tied to buttons

    // EXAMPLE: Reset the ahrs when button 3 on flight stick is pressed
    // new JoystickButton(flightStick, 3).toggleOnTrue(new
    // InstantCommand(ahrs::reset));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public DiffyDriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public DriveCommand getDriveCommand() {
    // driveCommand will run in teleop
    return driveCommand;
  }

  public IntakeCommand getIntakeCommand() {
    // intakeCommand will run in teleop
    return intakeCommand;
  }

  public DiffyDriveSubsystem getDriveSub() {
    // driveCommand will run in teleop
    return driveSubsystem;
  }

  public AutonCommand getAutonCommand() {
    // autonCommand will run in autonomous
    return autonCommand;
  }
}